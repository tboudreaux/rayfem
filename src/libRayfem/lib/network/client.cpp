#include "libRayfem/network/client.hpp"
#include "libRayfem/network/zmq.hpp"
#include "libRayfem/network/packet.hpp"
#include "czmq.h"

#include <string>

namespace {
    std::string format_address(const std::string& host, const size_t port) {
        return "tcp://" + host + ":" + std::to_string(port);
    }
}

namespace RayFEM {
    Client::Client(
        const std::string &host,
        const size_t port
    )
    :
    m_host(host),
    m_port(port),
    m_address(format_address(m_host, m_port)),
    m_socket(zsock_new_push(m_address.c_str())) {
        if (!m_socket) {
            throw std::runtime_error("Failed to create ZMQ socket to " + m_address);
        }
        zsock_set_linger(m_socket, 1000);
    }

    bool Client::send(mfem::Mesh &mesh, mfem::GridFunction &gf, const FrameMetadata& meta) {
        if (!m_socket) {
            return false;
        }

        std::stringstream m_ss, g_ss;

        mesh.Print(m_ss);
        gf.Save(g_ss);

        std::string m_data = m_ss.str();
        std::string g_data = g_ss.str();

        MsgPtr msg(zmsg_new());

        PacketHeader header{.metadata = meta, .mesh_bytes = m_data.size(), .gf_bytes = g_data.size()};
        zmsg_addmem(msg, &header, sizeof(PacketHeader));
        zmsg_addstr(msg, m_data.c_str());
        zmsg_addstr(msg, g_data.c_str());

        zmsg_t* raw_msg = msg.get();
        int rc = zmsg_send(&raw_msg, m_socket);
        msg.release();
        return rc == 0;


    }
}
