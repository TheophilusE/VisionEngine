
#pragma once

#include "SampleBase.hpp"
#include "BasicMath.hpp"

namespace Vision
{
using namespace Diligent;

class ApplicationBase : public SampleBase
{
public:
    ApplicationBase();
    ~ApplicationBase();

    virtual void ModifyEngineInitInfo(const ModifyEngineInitInfoAttribs& Attribs) override;
    virtual void Initialize(const SampleInitInfo& InitInfo) override;

    virtual void PreUpdate() override;
    virtual void FixedUpdate() override;
    virtual void PostUpdate() override;
    virtual void Update(double CurrTime, double ElapsedTime) override;
    virtual void Render() override;
    virtual void PostRender() override;
    virtual void WindowResize(Uint32 Width, Uint32 Height) override;

    virtual const Char* GetSampleName() const override { return "Vision Application"; }
};

} // namespace Vision