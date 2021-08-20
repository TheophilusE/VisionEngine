
#pragma once

#include "ApplicationBase.h"

namespace Vision
{
class VisionApp final : public ApplicationBase
{
public:
    ~VisionApp();

    virtual void Initialize(const SampleInitInfo& InitInfo) override final;

    virtual void PreUpdate() override final;
    virtual void FixedUpdate() override final;
    virtual void Update(double CurrTime, double ElapsedTime) override final;
    virtual void PostUpdate() override final;
    virtual void Render() override final;
    virtual void PostRender() override final;

    virtual const Char* GetSampleName() const override final { return "Vision Editor"; }

    virtual void WindowResize(Uint32 Width, Uint32 Height) override final;

private:
    void UpdateUI();
};
} // namespace Vision