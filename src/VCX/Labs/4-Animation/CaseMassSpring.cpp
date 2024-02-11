#include "Labs/4-Animation/CaseMassSpring.h"
#include "Engine/app.h"
#include "Labs/4-Animation/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Animation {

CaseMassSpring::CaseMassSpring()
    : _program(
          Engine::GL::UniqueProgram({Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                     Engine::GL::SharedShader("assets/shaders/flat.frag")})),
      _particlesItem(Engine::GL::VertexLayout()
                         .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
                     Engine::GL::PrimitiveType::Points),
      _springsItem(Engine::GL::VertexLayout()
                       .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
                   Engine::GL::PrimitiveType::Lines),
      _sphereItem(Engine::GL::VertexLayout()
                      .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Static, 0)
                      .Add<glm::vec3>("color", Engine::GL::DrawFrequency::Static, 1),
                  Engine::GL::PrimitiveType::Triangles) {
    _cameraManager.AutoRotate = false;
    _cameraManager.Save(_camera);
    ResetSystem(0);
    SetupSphere();
}

static int selectedMethod = 0;  // Initialize with the default scene index
static int selectedScene = 0;   // Initialize with the default scene index
static int point_num = 10;
static bool show_sphere = 1;
void CaseMassSpring::OnSetupPropsUI() {
    // ImGui checkbox for toggling sphere visibility
    ImGui::Checkbox("Show Sphere", &show_sphere);

    const char* methods[] = {"Fast Mass-Spring", "Explicit Euler", "Implicit Euler"};
    ImGui::Combo("Method", &selectedMethod, methods, IM_ARRAYSIZE(methods));

    // Selector for choosing initial scene
    const char* scenes[] = {"scene0, four fixed", "scene1, two fixed", "scene2, two fixed"};
    ImGui::Combo("Initial Scene", &selectedScene, scenes, IM_ARRAYSIZE(scenes));

    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Reset System")) {
            ResetSystem(selectedScene);
        }
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation"))
            _stopped = !_stopped;
        ImGui::SliderFloat("Part. Mass", &_massSpringSystem.Mass, .5f, 10.f);
        ImGui::SliderFloat("Spr. Stiff.", &_massSpringSystem.Stiffness, 10.f, 300.f);
        ImGui::SliderFloat("Spr. Damp.", &_massSpringSystem.Damping, .1f, 10.f);
        ImGui::SliderFloat("Gravity", &_massSpringSystem.Gravity, .1f, 1.f);
        ImGui::SliderInt("Spr. pointnum", &point_num, 10, 24);

        // Use selectedScene to determine the chosen scene and handle accordingly
    }
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Appearance")) {
        ImGui::SliderFloat("Part. Size", &_particleSize, 1, 6);
        ImGui::ColorEdit3("Part. Color", glm::value_ptr(_particleColor));
        ImGui::SliderFloat("Spr. Width", &_springWidth, .001f, 1.f);
        ImGui::ColorEdit3("Spr. Color", glm::value_ptr(_springColor));
    }
    ImGui::Spacing();
}

Common::CaseRenderResult CaseMassSpring::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
    if (!_stopped) {
        if (selectedMethod == 0) {
            FastMassSpringSystem(_massSpringSystem, Engine::GetDeltaTime(), show_sphere);
        } else if (selectedMethod == 1) {
            BasicMassSpringSystem(_massSpringSystem, Engine::GetDeltaTime());
        } else if (selectedMethod == 2) {
            AdvanceMassSpringSystem(_massSpringSystem, Engine::GetDeltaTime());
        }
    }

    _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));
    _springsItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));

    _frame.Resize(desiredSize);

    _cameraManager.Update(_camera);

    _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
    _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

    gl_using(_frame);
    glEnable(GL_LINE_SMOOTH);
    glPointSize(_particleSize);
    glLineWidth(_springWidth);

    _program.GetUniforms().SetByName("u_Color", _springColor);
    _springsItem.Draw({_program.Use()});
    _program.GetUniforms().SetByName("u_Color", _particleColor);
    _particlesItem.Draw({_program.Use()});

    glLineWidth(1.f);
    glPointSize(1.f);
    glDisable(GL_LINE_SMOOTH);

    // try this to render sphere
    if (show_sphere) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);

        // Set polygon mode to GL_LINE for wireframe rendering
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        _program.GetUniforms().SetByName("u_Color", glm::vec3(0.2f, 0.2f, 0.0f));
        _sphereItem.Draw({_program.Use()});

        // Restore polygon mode to GL_FILL for normal rendering
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    // end of rendering sphere

    return Common::CaseRenderResult{
        .Fixed = false,
        .Flipped = true,
        .Image = _frame.GetColorAttachment(),
        .ImageSize = desiredSize,
    };
}

void CaseMassSpring::OnProcessInput(ImVec2 const& pos) {
    _cameraManager.ProcessInput(_camera, pos);
}

void CaseMassSpring::ResetSystem(int scene_seq) {
    _massSpringSystem = {};
    // std::size_t const n = 22;
    std::size_t const n = point_num;

    // printf("@reset num is %d ", n);
    float const delta = 2.f / n;
    auto constexpr GetID = [](std::size_t const i, std::size_t const j, std::size_t const n) { return i * (n + 1) + j; };
    for (std::size_t i = 0; i <= n; i++) {
        for (std::size_t j = 0; j <= n; j++) {
            _massSpringSystem.AddParticle(glm::vec3(i * delta, 1.5f, j * delta - 1.f));
            if (i > 0)
                _massSpringSystem.AddSpring(GetID(i, j, n), GetID(i - 1, j, n));
            if (i > 1)
                _massSpringSystem.AddSpring(GetID(i, j, n), GetID(i - 2, j, n));
            if (j > 0)
                _massSpringSystem.AddSpring(GetID(i, j, n), GetID(i, j - 1, n));
            if (j > 1)
                _massSpringSystem.AddSpring(GetID(i, j, n), GetID(i, j - 2, n));
            if (i > 0 && j > 0)
                _massSpringSystem.AddSpring(GetID(i, j, n), GetID(i - 1, j - 1, n));
            if (i > 0 && j < n)
                _massSpringSystem.AddSpring(GetID(i, j, n), GetID(i - 1, j + 1, n));
        }
    }

    if (scene_seq == 0) {
        _massSpringSystem.Fixed[GetID(0, 0, n)] = true;
        _massSpringSystem.Fixed[GetID(0, n, n)] = true;
        _massSpringSystem.Fixed[GetID(n, 0, n)] = true;
        _massSpringSystem.Fixed[GetID(n, n, n)] = true;
    } else if (scene_seq == 1) {
        _massSpringSystem.Fixed[GetID(0, 0, n)] = true;
        _massSpringSystem.Fixed[GetID(0, n, n)] = true;
    } else if (scene_seq == 2) {
        _massSpringSystem.Fixed[GetID(0, 0, n)] = true;
        _massSpringSystem.Fixed[GetID(n, n, n)] = true;
    }

    std::vector<std::uint32_t> indices;
    for (auto const& spring : _massSpringSystem.Springs) {
        indices.push_back(std::uint32_t(spring.AdjIdx.first));
        indices.push_back(std::uint32_t(spring.AdjIdx.second));
    }
    _springsItem.UpdateElementBuffer(indices);
    setgotQ(false);
}

void CaseMassSpring::SetupSphere() {
    constexpr float radius = 1.0f;
    constexpr int numSegments = 100;

    std::vector<glm::vec3> sphereVertices;
    std::vector<glm::uvec3> sphereIndices;

    for (int i = 0; i <= numSegments; ++i) {
        for (int j = 0; j <= numSegments; ++j) {
            float theta = static_cast<float>(i) / numSegments * glm::pi<float>();
            float phi = static_cast<float>(j) / numSegments * glm::two_pi<float>();

            float x = radius * std::sin(theta) * std::cos(phi);
            float y = radius * std::sin(theta) * std::sin(phi);
            float z = radius * std::cos(theta);

            sphereVertices.push_back(glm::vec3(x, y, z));
        }
    }

    for (int i = 0; i < numSegments; ++i) {
        for (int j = 0; j < numSegments; ++j) {
            int vertexIndex = i * (numSegments + 1) + j;

            sphereIndices.push_back(glm::uvec3(vertexIndex, vertexIndex + 1, vertexIndex + numSegments + 1));
            sphereIndices.push_back(glm::uvec3(vertexIndex + 1, vertexIndex + numSegments + 2, vertexIndex + numSegments + 1));
        }
    }

    _sphereItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(sphereVertices));
    // _sphereItem.UpdateElementBuffer(Engine::make_span_bytes<glm::uvec3>(sphereIndices));
    // Convert glm::uvec3 to std::uint32_t
    std::vector<std::uint32_t> flatIndices;
    for (const auto& index : sphereIndices) {
        flatIndices.push_back(index.x);
        flatIndices.push_back(index.y);
        flatIndices.push_back(index.z);
    }

    // Now you can use flatIndices with UpdateElementBuffer
    _sphereItem.UpdateElementBuffer(flatIndices);
}

}  // namespace VCX::Labs::Animation
