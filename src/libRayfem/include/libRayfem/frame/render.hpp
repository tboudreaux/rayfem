#pragma once

#include "mfem.hpp"
#include "raylib.h"

#include <memory>

#include "libRayfem/network/packet.hpp"

namespace RayFEM {
    struct RenderFrame {
        FrameMetadata metadata;
        std::unique_ptr<mfem::Mesh> mesh;
        std::unique_ptr<mfem::GridFunction> gf;

        Model rayModel;
        bool isUploaded = false;
    };
}