#include "libRayfem/init.hpp"
#include "libRayfem/network/client.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc,char** argv) {
    if(argc!=2){std::cerr<<"Usage: viewer_lifecycle_test /path/to/RayFEM\n";return 1;}
    RayFEM::WindowParams window;window.port=5696;window.control_port=5697;
    window.executable=argv[1];window.timeout_ms=10000;
    try {
        auto mesh=mfem::Mesh::MakeCartesian3D(3,3,3,mfem::Element::HEXAHEDRON);
        mesh.SetCurvature(4);
        mfem::H1_FECollection collection(4,3);mfem::FiniteElementSpace space(&mesh,&collection);
        mfem::GridFunction field(&space);field=1;
        for(int cycle=0;cycle<2;++cycle) {
            const auto started=RayFEM::init(window);
            if(started.status!=RayFEM::InitStatus::Started)throw std::runtime_error("Viewer start: "+started.error);
            if(RayFEM::init(window).status!=RayFEM::InitStatus::AlreadyRunning)throw std::runtime_error("Viewer reuse failed");
            {
                RayFEM::Client client(window.host,window.port);
                if(!client.sendMesh(1,mesh))throw std::runtime_error("Mesh send failed");
                for(int i=0;i<8;++i) {
                    RayFEM::FrameMetadata frame;frame.frame_index=i;frame.sim_time=.1*i;
                    field=1+.1*i;
                    if(!client.sendFrame(1,{{"Temperature",&field}},frame))throw std::runtime_error("Frame send failed");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(700));
            }
            const auto stopped=RayFEM::shutdown(window);
            if(stopped.status!=RayFEM::ShutdownStatus::Stopped)throw std::runtime_error("Viewer close: "+stopped.error);
            if(RayFEM::shutdown(window).status!=RayFEM::ShutdownStatus::NotRunning)throw std::runtime_error("Repeated shutdown failed");
        }
        std::cout<<"PASS real viewer startup, reuse, graceful close, repeated shutdown and restart\n";
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
