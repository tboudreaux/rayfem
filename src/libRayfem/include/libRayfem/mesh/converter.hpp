#pragma once

#include "raylib.h"
#include "mfem.hpp"

namespace RayFEM {
    Model LoadModelFromMFEM(mfem::Mesh& mesh, mfem::GridFunction& gf);
}