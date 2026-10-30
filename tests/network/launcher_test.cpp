#include "libRayfem/init.hpp"
#include "libRayfem/network/server.hpp"
#include "libRayfem/network/zmq.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if defined(__APPLE__) || defined(__linux__)
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

namespace {
using namespace RayFEM;
using Clock=std::chrono::steady_clock;
void Check(bool okay,const std::string& message){if(!okay)throw std::runtime_error(message);}
template<class Function> void Wait(Function condition,const std::string& message,int milliseconds=3000) {
    const auto deadline=Clock::now()+std::chrono::milliseconds(milliseconds);
    while(!condition()){Check(Clock::now()<deadline,message);std::this_thread::sleep_for(std::chrono::milliseconds(5));}
}
void Ready(Server& server){Wait([&]{return server.ready()||server.getErrorCount();},"Server startup timeout");Check(server.ready(),server.getLastError());}

#if defined(__APPLE__) || defined(__linux__)
// A short-lived viewer stand-in uses the production Server handshake without
// initializing graphics. Only this test executable recognizes the helper path.
int ViewerHelper(int argc,char** argv) {
    std::map<std::string,std::string> options;
    for(int i=1;i+1<argc;i+=2)options[argv[i]]=argv[i+1];
    Check(options.size()==7,"Launcher must forward exactly seven viewer options");
    Server server(options.at("--listen"),std::stoi(options.at("--port")),std::stoi(options.at("--control-port")));
    Ready(server);
    if(const char* path=std::getenv("RAYFEM_TEST_RECORD")) {
        std::ofstream record(path);record<<getpid()<<'\n';
        for(const auto& [name,value]:options)record<<name<<'='<<value<<'\n';
    }
    if(const char* delay=std::getenv("RAYFEM_TEST_DELAY_MS"))std::this_thread::sleep_for(std::chrono::milliseconds(std::stoi(delay)));
    server.markViewerReady();
    const auto deadline=Clock::now()+std::chrono::seconds(8);
    while(!zsys_interrupted&&Clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return 0;
}
struct HelperRecord {
    std::filesystem::path path;
    explicit HelperRecord(std::filesystem::path value):path(std::move(value)) {setenv("RAYFEM_TEST_RECORD",path.c_str(),1);}
    ~HelperRecord(){std::ifstream input(path);pid_t pid=0;if(input>>pid&&pid>1)kill(pid,SIGTERM);unsetenv("RAYFEM_TEST_RECORD");}
    std::string text()const {std::ifstream input(path);return std::string(std::istreambuf_iterator<char>(input),{});}
};

int Run(int argc,char** argv) {
    const auto self=std::filesystem::canonical(argv[0]);
    if(argc>1&&std::string(argv[1])=="--listen")return ViewerHelper(argc,argv);
    if(argc>1&&std::string(argv[1])=="--parent-exit") {
        WindowParams options;options.port=std::stoi(argv[2]);options.executable=self;
        return init(options).status==InitStatus::Started?0:1;
    }
    const int base=16000+(getpid()%5000)*6;
    const auto directory=std::filesystem::temp_directory_path()/("rayfem-launcher-"+std::to_string(getpid()));
    std::filesystem::create_directories(directory);
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove_all(path,error);}}cleanup{directory};
    WindowParams options;options.port=base;options.timeout_ms=1000;
    options.executable=(directory/"missing-viewer").string();
    {auto invalid=options;invalid.control_port=invalid.port;Check(init(invalid).status==InitStatus::Failed,"Equal ports accepted");
     invalid=options;invalid.width=10;Check(init(invalid).status==InitStatus::Failed,"Invalid dimensions accepted");
     invalid=options;invalid.port=std::numeric_limits<int>::max();Check(init(invalid).status==InitStatus::Failed,"Extreme port accepted");
     invalid=options;invalid.port=65535;Check(init(invalid).status==InitStatus::Failed,"Port65535 needs an explicit control port");}
    {Server existing("127.0.0.1",base,base+1);Ready(existing);existing.markViewerReady();
     Check(init(options).status==InitStatus::AlreadyRunning,"Existing viewer should be reused without executable discovery");
     Check(existing.getErrorCount()==0,"Probes must not enter the frame parser");}
    options.port=base+2;
    {Server starting("127.0.0.1",options.port,options.port+1);Ready(starting);
     std::thread ready([&]{std::this_thread::sleep_for(std::chrono::milliseconds(180));starting.markViewerReady();});
     const auto result=init(options);ready.join();Check(result.status==InitStatus::AlreadyRunning,"Wait for an existing startup without launching");}
    options.port=base+4;
    {Server stalled("127.0.0.1",options.port,options.port+1);Ready(stalled);auto brief=options;brief.timeout_ms=250;
     const auto result=init(brief);Check(result.status==InitStatus::Failed&&result.error.find("existing")!=std::string::npos,"Existing startup timeout must report failure");}
    options.port=base+6;
    {Server occupied("127.0.0.1",options.port);Ready(occupied);const auto result=init(options);
     Check(result.status==InitStatus::Failed&&result.error.find("unavailable")!=std::string::npos,"A stream listener alone is not a viewer");}
    options.port=base+8;
    {std::atomic<bool> ready=false,stop=false;std::thread foreign([&]{SocketPtr socket(zsock_new(ZMQ_REP));zsock_set_linger(socket,0);zsock_set_rcvtimeo(socket,50);
        Check(zsock_bind(socket,"tcp://127.0.0.1:%d",options.port+1)>=0,"Foreign control bind");ready=true;
        while(!stop){FramePtr request(zframe_recv(socket));if(request)zstr_send(socket,"RAYFEM_READY 1 9999 wrongnonce");}});
     Wait([&]{return ready.load();},"Foreign endpoint startup");const auto result=init(options);stop=true;foreign.join();
     Check(result.status==InitStatus::Failed&&result.error.find("identify")!=std::string::npos,"Wrong port/nonce response accepted");}
    options.port=base+10;
    {const auto result=init(options);Check(result.status==InitStatus::Failed&&result.error.find("missing")!=std::string::npos,"Missing executable should be explained");}
    const auto literal_path=directory/"RayFEM viewer $(touch INJECTION) ; literal";
    std::filesystem::create_symlink(self,literal_path);
    {HelperRecord record(directory/"restart.txt");auto restart=options;restart.port=base;restart.executable=literal_path;restart.timeout_ms=3000;
     const auto result=init(restart);Check(result.status==InitStatus::Started,"Closed viewer ports should be reusable: "+result.error);}
    options.port=base+12;options.control_port=base+40;options.executable=literal_path;
    options.width=1064;options.height=704;options.fps=47;options.history=23;options.timeout_ms=3000;
    {HelperRecord record(directory/"launched.txt");
     // Concurrent callers share a launch and all observe a verified viewer.
     auto first=std::async(std::launch::async,[&]{return init(options);});
     auto second=std::async(std::launch::async,[&]{return init(options);});
     const auto a=first.get(),b=second.get();
     Check((a.status==InitStatus::Started&&b.status==InitStatus::AlreadyRunning)||(b.status==InitStatus::Started&&a.status==InitStatus::AlreadyRunning),"Concurrent init must start exactly one viewer: "+a.error+b.error);
     const auto arguments=record.text();
     Check(arguments.find("--width=1064")!=std::string::npos&&arguments.find("--height=704")!=std::string::npos&&arguments.find("--fps=47")!=std::string::npos&&arguments.find("--history=23")!=std::string::npos,"Window arguments were not preserved");
     Check(!std::filesystem::exists("INJECTION"),"Executable path was interpreted by a shell");
     Check(init(options).status==InitStatus::AlreadyRunning,"Repeat initialization changed viewer lifetime");}
    options.port=base+14;options.control_port=0;options.executable.clear();
    {HelperRecord record(directory/"environment.txt");setenv("RAYFEM_EXECUTABLE",literal_path.c_str(),1);
     const auto previous=std::signal(SIGCHLD,SIG_IGN);
     const auto result=init(options);std::signal(SIGCHLD,previous);unsetenv("RAYFEM_EXECUTABLE");
     Check(result.status==InitStatus::Started,"Environment discovery with caller-owned SIGCHLD disposition: "+result.error);}
    {HelperRecord record(directory/"survives-caller.txt");
     std::string port=std::to_string(base+16);std::vector<std::string> values{self,"--parent-exit",port};std::vector<char*> args;for(auto& value:values)args.push_back(value.data());args.push_back(nullptr);
     pid_t parent=0;Check(posix_spawn(&parent,self.c_str(),nullptr,nullptr,args.data(),environ)==0,"Test caller spawn");int status=0;Check(waitpid(parent,&status,0)==parent&&WIFEXITED(status)&&WEXITSTATUS(status)==0,"Test caller failed to launch viewer");
     auto after_exit=options;after_exit.port=base+16;after_exit.executable="/nonexistent";
     Check(init(after_exit).status==InitStatus::AlreadyRunning,"Viewer must survive the caller exiting");}
    options.port=base+18;options.executable="/usr/bin/false";
    {const auto result=init(options);Check(result.status==InitStatus::Failed&&result.error.find("exited")!=std::string::npos,"Early child exit must be reported");}
    options.port=base+20;options.executable.clear();
    {HelperRecord record(directory/"path.txt");
     const auto search_directory=directory/"search path";std::filesystem::create_directory(search_directory);std::filesystem::create_symlink(self,search_directory/"RayFEM");
     const char* prior=std::getenv("PATH");const bool had_path=prior;const std::string saved_path=prior?prior:"";
     setenv("PATH",search_directory.c_str(),1);const auto result=init(options);
     if(had_path)setenv("PATH",saved_path.c_str(),1);else unsetenv("PATH");
     Check(result.status==InitStatus::Started,"PATH executable discovery: "+result.error);}
    options.port=base+22;options.executable=literal_path;options.timeout_ms=100;
    {HelperRecord record(directory/"timeout-survival.txt");setenv("RAYFEM_TEST_DELAY_MS","350",1);
     const auto result=init(options);unsetenv("RAYFEM_TEST_DELAY_MS");
     Check(result.status==InitStatus::Failed&&result.error.find("left running")!=std::string::npos,"Short timeout must leave an in-progress launch running: "+result.error);
     Wait([&]{return std::filesystem::exists(record.path);},"Timed-out launcher did not start its helper");
     std::this_thread::sleep_for(std::chrono::milliseconds(400));
     options.timeout_ms=1000;options.executable="/nonexistent";
     Check(init(options).status==InitStatus::AlreadyRunning,"A viewer must survive its caller's startup timeout");}
    std::cout<<"Launcher checks passed: validated identity, startup/reuse, occupied ports, timeout, safe argv, concurrent initialization, environment discovery, and caller-independent viewer lifetime\n";
    return 0;
}
#endif
} // namespace

int main(int argc,char** argv){try{
#if defined(__APPLE__) || defined(__linux__)
    return Run(argc,argv);
#else
    std::cout<<"Launcher process tests require macOS/Linux\n";return 77;
#endif
}catch(const std::exception& error){std::cerr<<"Launcher test: "<<error.what()<<'\n';return 1;}}
