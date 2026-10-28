#pragma once

#include "raylib.h"
// Meson generates this in the build directory from the current SVG artwork,
// or copies the checked-in PNG data only after its input hash is verified.
#include <rayfem_rotation_icons_generated.hpp>
#include <array>

namespace RayFEM::UI {
// Load and unload inside the graphics context's lifetime. Artwork is embedded
// so installed viewers do not depend on the working directory or an SVG codec.
inline std::array<Texture2D,4> LoadRotationIcons() {
    std::array<Texture2D,4> textures{};
    for(size_t i=0;i<textures.size();++i) {
        const auto png=RotationArtwork::pngs[i];
        Image image=LoadImageFromMemory(".png",png.data(),static_cast<int>(png.size()));
        if(!image.data)continue;
        ImageFormat(&image,PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        // White RGB allows theme tinting while retaining the SVG's antialiasing.
        auto* pixels=static_cast<unsigned char*>(image.data);
        for(int pixel=0;pixel<image.width*image.height;++pixel)
            pixels[pixel*4]=pixels[pixel*4+1]=pixels[pixel*4+2]=255;
        textures[i]=LoadTextureFromImage(image);
        UnloadImage(image);
        SetTextureFilter(textures[i],TEXTURE_FILTER_BILINEAR);
    }
    return textures;
}
inline void UnloadRotationIcons(const std::array<Texture2D,4>& textures) {
    for(const auto texture:textures)if(texture.id)UnloadTexture(texture);
}
} // namespace RayFEM::UI
