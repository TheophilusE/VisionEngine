
#pragma once

#include "../Core/OS/OS.h"

namespace Vision
{

class Renderer
{
public:
    Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    virtual ~Renderer()                  = default;

    void Render();
};

} // namespace Vision