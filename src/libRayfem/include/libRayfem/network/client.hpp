#pragma once

#include <string>
#include "mfem.hpp"
#include "libRayfem/network/packet.hpp"
#include "zmq.hpp"

namespace RayFEM {
    class Client {
    public:
        Client(const std::string& host, size_t port);
        bool send(mfem::Mesh& mesh, mfem::GridFunction& gf, const FrameMetadata& meta);
    private:
        std::string m_host;
        size_t m_port;
        std::string m_address;

        SocketPtr m_socket;
    };
}
