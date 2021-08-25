
#include "Components.h"

namespace Vision
{

void BoundingBoxComponent::CreateBoundBoxPSO(TEXTURE_FORMAT RTVFmt, TEXTURE_FORMAT DSVFmt, Renderer& renderer)
{
    ShaderCreateInfo                               ShaderCI;
    RefCntAutoPtr<IShaderSourceInputStreamFactory> pShaderSourceFactory;
    renderer.GetEngineFactory()->CreateDefaultShaderSourceStreamFactory("shaders", &pShaderSourceFactory);
    ShaderCI.pShaderSourceStreamFactory = pShaderSourceFactory;
    ShaderCI.SourceLanguage             = SHADER_SOURCE_LANGUAGE_HLSL;
    ShaderCI.UseCombinedTextureSamplers = true;

    ShaderCI.Desc.ShaderType = SHADER_TYPE_VERTEX;
    ShaderCI.Desc.Name       = "BoundBox VS";
    ShaderCI.EntryPoint      = "BoundBoxVS";
    ShaderCI.FilePath        = "BoundBox.vsh";
    RefCntAutoPtr<IShader> pVS;
    renderer.GetRenderDevice()->CreateShader(ShaderCI, &pVS);

    ShaderCI.Desc.Name       = "BoundBox PS";
    ShaderCI.EntryPoint      = "BoundBoxPS";
    ShaderCI.FilePath        = "BoundBox.psh";
    ShaderCI.Desc.ShaderType = SHADER_TYPE_PIXEL;
    RefCntAutoPtr<IShader> pPS;
    renderer.GetRenderDevice()->CreateShader(ShaderCI, &pPS);


    GraphicsPipelineStateCreateInfo PSOCreateInfo;
    PipelineStateDesc&              PSODesc          = PSOCreateInfo.PSODesc;
    GraphicsPipelineDesc&           GraphicsPipeline = PSOCreateInfo.GraphicsPipeline;

    PSODesc.Name = "BoundBox PSO";

    GraphicsPipeline.NumRenderTargets = 1;
    GraphicsPipeline.RTVFormats[0]    = RTVFmt;
    GraphicsPipeline.DSVFormat        = DSVFmt;

    PSOCreateInfo.pVS = pVS;
    PSOCreateInfo.pPS = pPS;

    GraphicsPipeline.RTVFormats[0]              = renderer.GetSwapChain()->GetDesc().ColorBufferFormat;
    GraphicsPipeline.NumRenderTargets           = 1;
    GraphicsPipeline.DSVFormat                  = renderer.GetSwapChain()->GetDesc().DepthBufferFormat;
    GraphicsPipeline.PrimitiveTopology          = PRIMITIVE_TOPOLOGY_LINE_LIST;
    GraphicsPipeline.DepthStencilDesc.DepthFunc = COMPARISON_FUNC_LESS_EQUAL;

    renderer.GetRenderDevice()->CreateGraphicsPipelineState(PSOCreateInfo, &m_BoundBoxPSO);
    m_BoundBoxPSO->GetStaticVariableByName(SHADER_TYPE_VERTEX, "cbCameraAttribs")->Set(renderer.GetCamAttribs());
    m_BoundBoxPSO->CreateShaderResourceBinding(&m_BoundBoxSRB, true);
}

void MeshComponent::LoadModel(const char* Path, Renderer& renderer)
{
    if (m_Model)
    {
        m_PlayAnimation  = false;
        m_AnimationIndex = 0;
        m_AnimationTimers.clear();
    }

    GLTF::Model::CreateInfo ModelCI;
    ModelCI.FileName   = Path;
    ModelCI.pCacheInfo = m_bUseResourceCache ? &m_CacheUseInfo : nullptr;
    m_Model.reset(new GLTF::Model{renderer.GetRenderDevice(), renderer.GetDeviceContext(), ModelCI});

    m_ModelResourceBindings = renderer.CreateResourceBindings(*m_Model, renderer.GetCamAttribs(), renderer.GetLightAttribs());

    // Center and scale model
    float3 ModelDim{m_Model->AABBTransform[0][0], m_Model->AABBTransform[1][1], m_Model->AABBTransform[2][2]};
    float  Scale     = (1.0f / std::max(std::max(ModelDim.x, ModelDim.y), ModelDim.z)) * 0.5f;
    auto   Translate = -float3(m_Model->AABBTransform[3][0], m_Model->AABBTransform[3][1], m_Model->AABBTransform[3][2]);
    Translate += -0.5f * ModelDim;
    float4x4 InvYAxis = float4x4::Identity();
    InvYAxis._22      = -1;

    auto ModelTransform = float4x4::Translation(Translate) * float4x4::Scale(Scale) * InvYAxis;
    m_Model->Transform(ModelTransform);

    if (!m_Model->Animations.empty())
    {
        m_AnimationTimers.resize(m_Model->Animations.size());
        m_AnimationIndex = 0;
        m_PlayAnimation  = true;
    }

    m_CameraId = 0;
    m_Cameras.clear();
    for (const auto* node : m_Model->LinearNodes)
    {
        if (node->pCamera && node->pCamera->Type == GLTF::Camera::Projection::Perspective)
            m_Cameras.push_back(node->pCamera.get());
    }
}
} // namespace Vision