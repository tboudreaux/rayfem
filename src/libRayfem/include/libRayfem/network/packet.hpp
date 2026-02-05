#pragma once

#include <cstdint>

namespace RayFEM {
    struct FrameMetadata {
        uint32_t frame_index = 0;
        double sim_time = 0;
        char label[64] = "MFEM Simulation";
    };

#pragma pack(push, 1)
    struct PacketHeader {
        uint64_t magic = 0x52415946454D;
        FrameMetadata metadata;
        uint64_t mesh_bytes = 0;
        uint64_t gf_bytes = 0;
    };
#pragma pack(pop)

}