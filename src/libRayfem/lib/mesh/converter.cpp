#include "libRayfem/mesh/converter.hpp"
#include "mfem.hpp"
#include "raylib.h"

namespace {
    Color GetColorFromValue(double val, double min, double max) {
        double t = (val - min) / (max - min);
        t = std::clamp(t, 0.0, 1.0);
        return ColorFromNormalized({t, 1-t, 0.5+(t*0.5), 1};
    }
}

namespace RayFEM {
    Model LoadModelFromMFEM(mfem::Mesh &mesh, mfem::GridFunction &gf) {
        Mesh rMesh = { 0 };
        int numElements = mesh.GetNE();

        int totalTriangles = 0;
        for (int i = 0; i < numElements; i++) {
            mfem::Geometry::Type geom = mesh.GetElementBaseGeometry(i);
            switch (geom) {
                case mfem::Geometry::TRIANGLE:
                    totalTriangles += 1;
                    break;
                case mfem::Geometry::SQUARE:
                    totalTriangles += 2;
                    break;
                case mfem::Geometry::TETRAHEDRON:
                    totalTriangles += 4;
                    break;
                case mfem::Geometry::CUBE:
                    totalTriangles += 12;
                    break;
                default:
                    throw std::runtime_error("Unsupported element geometry in MFEM to Raylib converter.");
            }
        }
        rMesh.triangleCount = totalTriangles;

        rMesh.vertexCount = rMesh.triangleCount * 3;

        rMesh.vertices = (float *)RL_MALLOC(rMesh.vertexCount * 3 * sizeof(float));
        rMesh.colors = (unsigned char *)RL_MALLOC(rMesh.vertexCount * 4 * sizeof(unsigned char));

        double vMin = gf.Min();
        double vMax = gf.Max();


        int vOffset =0;
        auto AddTriangle = [&](const mfem::Vector& p1, const mfem::Vector& p2, const mfem::Vector& p3, double val1, double val2, double val3) {
            float* v = rMesh.vertices;

            v[vOffset * 3 + 0] = static_cast<float>(p1(0));
            v[vOffset * 3 + 1] = static_cast<float>(p1(1));
            v[vOffset * 3 + 2] = static_cast<float>(p1(2));

            v[vOffset * 3 + 3] = static_cast<float>(p2(0));
            v[vOffset * 3 + 4] = static_cast<float>(p2(1));
            v[vOffset * 3 + 5] = static_cast<float>(p2(2));

            v[vOffset * 3 + 6] = static_cast<float>(p3(0));
            v[vOffset * 3 + 7] = static_cast<float>(p3(1));
            v[vOffset * 3 + 8] = static_cast<float>(p3(2));

            const Color c1 = GetColorFromValue(val1, vMin, vMax);
            const Color c2 = GetColorFromValue(val2, vMin, vMax);
            const Color c3 = GetColorFromValue(val3, vMin, vMax);

            unsigned char* c = rMesh.colors;
            c[vOffset * 4 + 0] = c1.r;
            c[vOffset * 4 + 1] = c1.g;
            c[vOffset * 4 + 2] = c1.b;
            c[vOffset * 4 + 3] = c1.a;

            c[vOffset * 4 + 4] = c2.r;
            c[vOffset * 4 + 5] = c2.g;
            c[vOffset * 4 + 6] = c2.b;
            c[vOffset * 4 + 7] = c2.a;

            c[vOffset * 4 + 8] = c3.r;
            c[vOffset * 4 + 9] = c3.g;
            c[vOffset * 4 + 10] = c3.b;
            c[vOffset * 4 + 11] = c3.a;

            vOffset += 3;
        };

        for (int i = 0; i < numElements; i++) {
            mfem::Element* el = mesh.GetElement(i);
            mfem::Array<int> verticies;
            mesh.GetElementVertices(i, verticies);

            auto GetPV = [&](int localIdx) {
                mfem::Vector pos;
                pos = mesh.GetVertex(verticies[localIdx]);

                if (pos.Size() == 2) {
                    pos.SetSize(3);
                    pos(2) = 0.0;
                }
                return std::make_pair(pos, gf(verticies[localIdx]));
            };

            mfem::Geometry::Type geom = mesh.GetElementBaseGeometry(i);
            switch (geom) {
                case mfem::Geometry::TRIANGLE: {
                    auto [p0, v0] = GetPV(0);
                    auto [p1, v1] = GetPV(1);
                    auto [p2, v2] = GetPV(2);

                    AddTriangle(p0, p1, p2, v0, v1, v2);

                }
            }

        }
    }
}
