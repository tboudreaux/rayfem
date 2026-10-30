#include "libRayfem/init.hpp"
#include "libRayfem/network/zmq.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

#if defined(__APPLE__) || defined(__linux__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

namespace RayFEM {
namespace {
using Clock = std::chrono::steady_clock;
enum class Reply { Absent, Starting, Ready, Other };

std::mutex& LifecycleMutex() {
    static std::mutex mutex;
    return mutex;
}

std::string Nonce() {
    static std::atomic<unsigned long long> counter{0};
    std::ostringstream value;
    value << std::hex << Clock::now().time_since_epoch().count() << ++counter;
    return value.str();
}

std::string Address(const std::string& host,int port) {
    return "tcp://" + (host.find(':') != std::string::npos ? "["+host+"]" : host) + ":" + std::to_string(port);
}

Reply Probe(const std::string& host,int stream_port,int control_port,int timeout_ms) {
    const std::string nonce=Nonce();
    const std::string suffix=" 1 "+std::to_string(stream_port)+" "+nonce;
    SocketPtr socket(zsock_new(ZMQ_REQ));
    if (!socket) return Reply::Absent;
    zsock_set_linger(socket,0);
    zsock_set_sndtimeo(socket,timeout_ms);
    zsock_set_rcvtimeo(socket,timeout_ms);
    zsock_set_maxmsgsize(socket,256);
    zsock_set_ipv6(socket,1);
    if(zsock_connect(socket,"%s",Address(host,control_port).c_str())<0 ||
       zstr_send(socket,("RAYFEM_PROBE"+suffix).c_str())<0) return Reply::Absent;
    FramePtr frame(zframe_recv(socket));
    if(!frame) return Reply::Absent;
    if(zsock_rcvmore(socket)) return Reply::Other;
    const std::string response(reinterpret_cast<const char*>(zframe_data(frame)),zframe_size(frame));
    if(response=="RAYFEM_READY"+suffix) return Reply::Ready;
    if(response=="RAYFEM_STARTING"+suffix) return Reply::Starting;
    return Reply::Other;
}

// Ask the control endpoint for one reply line; nullopt when nothing answers.
std::optional<std::string> Ask(const std::string& host,int control_port,const std::string& request,int timeout_ms) {
    SocketPtr socket(zsock_new(ZMQ_REQ));
    if (!socket) return std::nullopt;
    zsock_set_linger(socket,0);
    zsock_set_sndtimeo(socket,timeout_ms);
    zsock_set_rcvtimeo(socket,timeout_ms);
    zsock_set_maxmsgsize(socket,256);
    zsock_set_ipv6(socket,1);
    if(zsock_connect(socket,"%s",Address(host,control_port).c_str())<0 || zstr_send(socket,request.c_str())<0) return std::nullopt;
    FramePtr frame(zframe_recv(socket));
    if(!frame || zsock_rcvmore(socket)) return std::nullopt;
    return std::string(reinterpret_cast<const char*>(zframe_data(frame)),zframe_size(frame));
}

bool IsLocal(const std::string& host) {
    return host=="localhost" || host=="127.0.0.1" || host=="::1";
}

#if defined(__APPLE__) || defined(__linux__)
std::string PortError(const std::string& host,int port) {
    const bool ipv6=host=="::1";
    const int descriptor=socket(ipv6?AF_INET6:AF_INET,SOCK_STREAM,0);
    if(descriptor<0) return std::string("Cannot check local port: ")+std::strerror(errno);
    // Match the viewer's restart behavior after established connections leave
    // TIME_WAIT entries. listen() still rejects an active listener.
    const int reuse=1;setsockopt(descriptor,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
    int result;
    if(ipv6) {
        sockaddr_in6 address{};address.sin6_family=AF_INET6;address.sin6_port=htons(port);address.sin6_addr=in6addr_loopback;
        result=bind(descriptor,reinterpret_cast<sockaddr*>(&address),sizeof(address));
    } else {
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        result=bind(descriptor,reinterpret_cast<sockaddr*>(&address),sizeof(address));
    }
    if(result==0)result=listen(descriptor,1);
    const int error=errno;close(descriptor);
    if(result==0) return {};
    return "Local port "+std::to_string(port)+" is unavailable and no compatible RayFEM viewer answered: "+std::strerror(error);
}

std::filesystem::path CurrentExecutable() {
#if defined(__APPLE__)
    std::uint32_t size=0;_NSGetExecutablePath(nullptr,&size);
    std::vector<char> buffer(size);
    if(_NSGetExecutablePath(buffer.data(),&size)==0) return std::filesystem::weakly_canonical(buffer.data());
#elif defined(__linux__)
    std::vector<char> buffer(4096);
    const auto count=readlink("/proc/self/exe",buffer.data(),buffer.size()-1);
    if(count>0 && count<static_cast<ssize_t>(buffer.size()-1)) return std::filesystem::path(std::string(buffer.data(),count));
#endif
    return {};
}

std::string ResolveExecutable(const std::string& requested) {
    namespace fs=std::filesystem;
    auto usable=[](const fs::path& path) {
        std::error_code error;
        return fs::is_regular_file(path,error) && access(path.c_str(),X_OK)==0;
    };
    if(requested.find('/')!=std::string::npos) {
        return usable(requested)?fs::absolute(requested).string():std::string{};
    }
    const char* environment=std::getenv("PATH");
    if(!environment) return {};
    std::istringstream paths(environment);std::string directory;
    while(std::getline(paths,directory,':')) {
        const auto candidate=fs::path(directory.empty()?".":directory)/requested;
        if(usable(candidate)) return fs::absolute(candidate).string();
    }
    return {};
}

std::string FindExecutable(const WindowParams& parameters,std::string& error) {
    if(!parameters.executable.empty()) {
        const auto path=ResolveExecutable(parameters.executable);
        if(path.empty()) error="The requested RayFEM executable is missing or is not executable: "+parameters.executable;
        return path;
    }
    if(const char* configured=std::getenv("RAYFEM_EXECUTABLE");configured && *configured) {
        const auto path=ResolveExecutable(configured);
        if(path.empty()) error="RAYFEM_EXECUTABLE does not name an executable: "+std::string(configured);
        return path;
    }
    const auto current=CurrentExecutable();
    if(!current.empty()) {
        const auto path=ResolveExecutable((current.parent_path()/"RayFEM").string());
        if(!path.empty()) return path;
    }
    const auto path=ResolveExecutable("RayFEM");
    if(path.empty()) error="Cannot find RayFEM. Set WindowParams.executable or RAYFEM_EXECUTABLE, place it beside the caller, or add it to PATH.";
    return path;
}

struct Child {std::atomic<bool> exited=false;std::atomic<int> status=0;};
std::shared_ptr<Child> Launch(const std::string& executable,const WindowParams& parameters,
                            const std::string& host,int control_port,std::string& error) {
    std::vector<std::string> arguments{executable,"--listen",host,"--port",std::to_string(parameters.port),
        "--control-port",std::to_string(control_port),"--width",std::to_string(parameters.width),
        "--height",std::to_string(parameters.height),"--fps",std::to_string(parameters.fps),
        "--history",std::to_string(parameters.history)};
    std::vector<char*> argv;for(auto& value:arguments)argv.push_back(value.data());argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    int code=posix_spawn_file_actions_init(&actions);
    if(code) {error="Cannot prepare RayFEM process: "+std::string(std::strerror(code));return {};}
    for(int descriptor=0;descriptor<3&&!code;++descriptor)
        code=posix_spawn_file_actions_addopen(&actions,descriptor,"/dev/null",descriptor==0?O_RDONLY:O_WRONLY,0);
    posix_spawnattr_t attributes;
    const int initialized=posix_spawnattr_init(&attributes);
    if(!code)code=initialized;
    if(!code)code=posix_spawnattr_setpgroup(&attributes,0);
    if(!code)code=posix_spawnattr_setflags(&attributes,POSIX_SPAWN_SETPGROUP);
    pid_t pid=0;
    if(!code)code=posix_spawn(&pid,executable.c_str(),&actions,&attributes,argv.data(),environ);
    if(!initialized)posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    if(code) {error="Cannot start RayFEM: "+std::string(std::strerror(code));return {};}
    const auto child=std::make_shared<Child>();
    std::thread([child,pid] {
        int status=0;pid_t result;
        do {result=waitpid(pid,&status,0);} while(result<0&&errno==EINTR);
        // A consuming application may ignore SIGCHLD or reap children itself.
        // ECHILD means monitoring is unavailable, not that the viewer exited.
        if(result==pid){child->status=status;child->exited=true;}
    }).detach();
    return child;
}
#endif
} // namespace

InitResult init(const WindowParams& parameters) {
    KeepHostSignals();
    const std::lock_guard lock(LifecycleMutex());
    InitResult result;
    try {
        if(parameters.port<1 || parameters.port>65535)
            {result.error="The stream port must be from 1 to 65535.";return result;}
        const int control_port=parameters.control_port?parameters.control_port:parameters.port+1;
        if(control_port<1 || control_port>65535 || control_port==parameters.port)
            {result.error="Stream and control ports must be distinct values from 1 to 65535.";return result;}
        if(parameters.host.empty() || parameters.host.find_first_of(" \t\r\n/[]")!=std::string::npos || parameters.host.find('\0')!=std::string::npos || parameters.executable.find('\0')!=std::string::npos)
            {result.error="Supply a host name or IP address without a URL or brackets.";return result;}
        if(parameters.width<960 || parameters.width>7680 || parameters.height<640 || parameters.height>4320 ||
           parameters.fps<1 || parameters.fps>240 || parameters.history<1 || parameters.history>10000 ||
           parameters.timeout_ms<100 || parameters.timeout_ms>60000)
            {result.error="Invalid window, frame-rate, history, or timeout setting.";return result;}
        const auto deadline=Clock::now()+std::chrono::milliseconds(parameters.timeout_ms);
        const std::string host=parameters.host=="localhost"?"127.0.0.1":parameters.host;
        auto probe=[&](bool discovery=false) {
            const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
            // Leave time for spawning/readiness even with the shortest allowed
            // timeout; the first unanswered probe must not consume it all.
            return Probe(host,parameters.port,control_port,std::clamp<int>(static_cast<int>(discovery?remaining/3:remaining),1,150));
        };
        auto reply=probe(true);
        if(reply==Reply::Ready) {result.status=InitStatus::AlreadyRunning;return result;}
        if(reply==Reply::Other) {result.error="The control endpoint did not identify itself as a compatible RayFEM viewer.";return result;}
        bool already_starting=reply==Reply::Starting;
#if defined(__APPLE__) || defined(__linux__)
        std::shared_ptr<Child> child;
        if(!already_starting) {
            if(!IsLocal(host)) {result.error="No compatible remote viewer responded. Automatic launching is restricted to localhost.";return result;}
            result.error=PortError(host,parameters.port);
            if(result.error.empty())result.error=PortError(host,control_port);
            if(!result.error.empty()) {
                // Another process can start a viewer between our first probe
                // and the port checks. Verify its service identity rather
                // than treating that startup race as failure.
                reply=probe();
                if(reply==Reply::Ready){result.error.clear();result.status=InitStatus::AlreadyRunning;return result;}
                if(reply==Reply::Starting){already_starting=true;result.error.clear();}
                else return result;
            }
            if(!already_starting) {
                result.executable=FindExecutable(parameters,result.error);if(result.executable.empty())return result;
                if(Clock::now()>=deadline){result.error="Startup timeout expired before RayFEM could be launched.";return result;}
                child=Launch(result.executable,parameters,host,control_port,result.error);if(!child)return result;
            }
        }
#else
        if(!already_starting) {result.error="Automatic RayFEM launching is supported on macOS and Linux. Start a viewer manually on this platform.";return result;}
#endif
        while(Clock::now()<deadline) {
            reply=probe();
            if(reply==Reply::Ready) {
                bool reused=already_starting;
#if defined(__APPLE__) || defined(__linux__)
                // A simultaneous caller may have won the listener bind. Its
                // viewer is reusable even when our launched process exited.
                reused=reused || (child && child->exited);
#endif
                result.status=reused?InitStatus::AlreadyRunning:InitStatus::Started;return result;
            }
            if(reply==Reply::Other) {result.error="The control endpoint returned an incompatible RayFEM handshake.";return result;}
#if defined(__APPLE__) || defined(__linux__)
            if(child && child->exited && reply!=Reply::Starting) {result.error="RayFEM exited before its viewer became ready (process status "+std::to_string(child->status.load())+").";return result;}
#endif
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
        result.error=already_starting?"The existing RayFEM viewer did not become ready before the timeout.":"RayFEM did not become ready before the timeout; any launched process has been left running.";
    } catch(const std::exception& error) {result.error=error.what();}
    return result;
}

ShutdownResult shutdown(const WindowParams& parameters) {
    KeepHostSignals();
    const std::lock_guard lock(LifecycleMutex());
    ShutdownResult result;
    try {
        if (parameters.port < 1 || parameters.port > 65535) {
            result.error = "The stream port must be from 1 to 65535."; return result;
        }
        const int control_port = parameters.control_port ? parameters.control_port : parameters.port + 1;
        if (control_port < 1 || control_port > 65535 || control_port == parameters.port) {
            result.error = "Stream and control ports must be distinct values from 1 to 65535."; return result;
        }
        if (parameters.host.empty() || parameters.host.find_first_of(" \t\r\n/[]") != std::string::npos ||
            parameters.host.find('\0') != std::string::npos) {
            result.error = "Supply a host name or IP address without a URL or brackets."; return result;
        }
        if (parameters.timeout_ms < 100 || parameters.timeout_ms > 60000) {
            result.error = "Shutdown timeout must be from 100 to 60000 milliseconds."; return result;
        }
        const auto deadline = Clock::now() + std::chrono::milliseconds(parameters.timeout_ms);
        const std::string host = parameters.host == "localhost" ? "127.0.0.1" : parameters.host;
        const auto remaining = [&] {
            return std::max(0, static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - Clock::now()).count()));
        };
        SocketPtr socket(zsock_new(ZMQ_REQ));
        if (!socket) { result.error = "Cannot create the viewer control socket."; return result; }
        zsock_set_linger(socket, 0);
        zsock_set_ipv6(socket, 1);
        zsock_set_maxmsgsize(socket, 256);
        if (zsock_connect(socket, "%s", Address(host, control_port).c_str()) < 0) {
            result.error = "Cannot connect to the viewer control endpoint."; return result;
        }
        auto exchange = [&](const std::string& request, int budget) -> std::optional<std::string> {
            const auto end = Clock::now() + std::chrono::milliseconds(std::min(budget, remaining()));
            const auto timeout = [&] { return std::max(1, static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(end - Clock::now()).count())); };
            zsock_set_sndtimeo(socket, timeout());
            if (zstr_send(socket, request.c_str()) < 0) return std::nullopt;
            zsock_set_rcvtimeo(socket, timeout());
            FramePtr reply(zframe_recv(socket));
            if (!reply) return std::nullopt;
            if (zsock_rcvmore(socket)) return std::string{};
            return std::string(reinterpret_cast<const char*>(zframe_data(reply)), zframe_size(reply));
        };
        const std::string suffix = " 1 " + std::to_string(parameters.port) + " " + Nonce();
        const auto identity = exchange("RAYFEM_PROBE" + suffix, std::min(500, parameters.timeout_ms / 2));
        if (!identity) {
#if defined(__APPLE__) || defined(__linux__)
            if (IsLocal(host) && PortError(host, parameters.port).empty() && PortError(host, control_port).empty()) {
                result.status = ShutdownStatus::NotRunning; return result;
            }
#endif
            result.error = "No compatible RayFEM viewer answered at the requested endpoint."; return result;
        }
        if (*identity != "RAYFEM_READY" + suffix && *identity != "RAYFEM_STARTING" + suffix) {
            result.error = "The control endpoint did not identify itself as a compatible RayFEM viewer."; return result;
        }

        // Keep the verified connection, and watch its actual disconnect. A
        // temporary unanswered probe must not be mistaken for completed exit.
        const std::string monitor_address = "inproc://rayfem-shutdown-" + Nonce();
        if (zmq_socket_monitor(zsock_resolve(socket), monitor_address.c_str(), ZMQ_EVENT_DISCONNECTED) < 0) {
            result.error = "Cannot monitor completion of the viewer shutdown."; return result;
        }
        SocketPtr monitor(zsock_new(ZMQ_PAIR));
        if (!monitor) { result.error = "Cannot create the shutdown monitor."; return result; }
        zsock_set_linger(monitor, 0);
        if (zsock_connect(monitor, "%s", monitor_address.c_str()) < 0) {
            result.error = "Cannot connect the shutdown monitor."; return result;
        }
        const std::string stop_suffix = " 1 " + std::to_string(parameters.port) + " " + Nonce();
        const auto acknowledgement = exchange("RAYFEM_SHUTDOWN" + stop_suffix, remaining());
        if (!acknowledgement || *acknowledgement != "RAYFEM_STOPPING" + stop_suffix) {
            result.error = "The viewer did not acknowledge shutdown; it may be an older version or still be running.";
            return result;
        }
        while (remaining() > 0) {
            zsock_set_rcvtimeo(monitor, remaining());
            FramePtr event(zframe_recv(monitor));
            if (!event) break;
            uint16_t type = 0;
            if (zframe_size(event) >= sizeof(type)) std::memcpy(&type, zframe_data(event), sizeof(type));
            while (zsock_rcvmore(monitor)) { FramePtr address(zframe_recv(monitor)); if (!address) break; }
            if (type == ZMQ_EVENT_DISCONNECTED) { result.status = ShutdownStatus::Stopped; return result; }
        }
        result.error = "RayFEM acknowledged shutdown but did not finish closing before the timeout; no process was killed.";
    } catch (const std::exception& error) { result.error = error.what(); }
    return result;
}
ViewerStats stats(const WindowParams& parameters) {
    KeepHostSignals();
    ViewerStats result;
    if (parameters.port < 1 || parameters.port > 65535) { result.error = "Invalid viewer port."; return result; }
    const int control_port = parameters.control_port ? parameters.control_port : parameters.port + 1;
    if (control_port < 1 || control_port > 65535) { result.error = "Invalid viewer control port."; return result; }
    const std::string host = parameters.host == "localhost" ? "127.0.0.1" : parameters.host;
    const std::string prefix = "RAYFEM_STATS 1 " + std::to_string(parameters.port) + " " + Nonce();
    const auto reply = Ask(host, control_port, prefix, std::max(1, parameters.timeout_ms));
    if (!reply) { result.error = "No RayFEM viewer answered at the requested endpoint."; return result; }
    std::istringstream fields(reply->starts_with(prefix + " ") ? reply->substr(prefix.size() + 1) : std::string());
    std::string extra;
    if (!(fields >> result.received >> result.errors >> result.dropped >> result.skipped) || (fields >> extra))
        result.error = "The viewer did not report statistics; it may be an older version.";
    return result;
}

} // namespace RayFEM
