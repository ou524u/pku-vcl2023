#include "Labs/4-Animation/tasks.h"
#include <spdlog/spdlog.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include "CustomFunc.inl"
#include "IKSystem.h"

namespace VCX::Labs::Animation {
void ForwardKinematics(IKSystem& ik, int StartIndex) {
    if (StartIndex == 0) {
        ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
        ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
        StartIndex = 1;
    }

    for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
        // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
        ik.JointGlobalRotation[i] = ik.JointLocalRotation[i] * ik.JointGlobalRotation[i - 1];

        // glm::vec3 rotatedOffset = glm::rotate(ik.JointGlobalRotation[i], ik.JointLocalOffset[i]);
        // ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + rotatedOffset;

        ik.JointGlobalPosition[i] = ik.JointLocalOffset[i] + ik.JointGlobalPosition[i - 1];
    }
}

void InverseKinematicsCCD(IKSystem& ik, const glm::vec3& EndPosition, int maxCCDIKIteration, float eps) {
    ForwardKinematics(ik, 0);

    // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
    for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
        // your code here: ccd ik
        for (int i = ik.JointLocalOffset.size() - 1; i > 0; i--) {
            glm::vec3 d1 = glm::normalize(EndPosition - ik.JointGlobalPosition[i - 1]);
            glm::vec3 d2 = glm::normalize(ik.JointGlobalPosition.back() - ik.JointGlobalPosition[i - 1]);
            ik.JointLocalRotation[i] = glm::rotation(d2, d1) * ik.JointLocalRotation[i];
            ik.JointLocalOffset[i] = glm::rotation(d2, d1) * ik.JointLocalOffset[i];
            ForwardKinematics(ik, i - 1);
        }
    }
}

void InverseKinematicsFABR(IKSystem& ik, const glm::vec3& EndPosition, int maxFABRIKIteration, float eps) {
    ForwardKinematics(ik, 0);
    int nJoints = ik.NumJoints();
    std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
    for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
        // task: fabr ik
        // backward update
        glm::vec3 next_position = EndPosition;
        backward_positions[nJoints - 1] = EndPosition;

        for (int i = nJoints - 2; i >= 0; i--) {
            // your code here
            glm::vec3 back_dir = glm::normalize(ik.JointGlobalPosition[i] - backward_positions[i + 1]);

            backward_positions[i] = backward_positions[i + 1] + back_dir * ik.JointOffsetLength[i + 1];
        }

        // forward update
        glm::vec3 now_position = ik.JointGlobalPosition[0];
        forward_positions[0] = ik.JointGlobalPosition[0];
        for (int i = 1; i < nJoints; i++) {
            // your code here
            glm::vec3 forv_dir = glm::normalize(backward_positions[i] - forward_positions[i - 1]);
            forward_positions[i] = forward_positions[i - 1] + forv_dir * ik.JointOffsetLength[i];
            ik.JointLocalOffset[i] = forward_positions[i] - forward_positions[i - 1];
        }
        ik.JointGlobalPosition = forward_positions;  // copy forward positions to joint_positions
    }

    // Compute joint rotation by position here.
    for (int i = 0; i < nJoints - 1; i++) {
        ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
    }
    ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
    for (int i = 1; i < nJoints - 1; i++) {
        ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
    }
    ForwardKinematics(ik, 0);
}

IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
    // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
    int nums = 5000;
    using Vec3Arr = std::vector<glm::vec3>;
    std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
    int index = 0;
    for (int i = 0; i < nums; i++) {
        // float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
        // float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
        float x_val = 5.0e-4f * face_x(92 * glm::pi<float>() * i / nums) + 1.0f;
        float y_val = 5.0e-4f * face_y(92 * glm::pi<float>() * i / nums);
        if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3)
            continue;
        (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
    }
    custom->resize(index);
    return custom;
}

static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const& glm_v) {
    Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const*>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
    return v;
}

static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const& eigen_v) {
    return std::vector<glm::vec3>(
        reinterpret_cast<glm::vec3 const*>(eigen_v.data()),
        reinterpret_cast<glm::vec3 const*>(eigen_v.data() + eigen_v.size()));
}

static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const& triplets) {
    Eigen::SparseMatrix<float> matLinearized(n, n);
    matLinearized.setFromTriplets(triplets.begin(), triplets.end());
    return matLinearized;
}

// solve Ax = b and return x
static Eigen::VectorXf ComputeSimplicialLLT(
    Eigen::SparseMatrix<float> const& A,
    Eigen::VectorXf const& b) {
    auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
    return solver.solve(b);
}

void AdvanceMassSpringSystem(MassSpringSystem& system, float const dt) {
    // your code here: rewrite following code

    // int const steps = 1000;
    // float const ddt = dt / steps;
    // for (std::size_t s = 0; s < steps; s++) {
    //     std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0.0f, -system.Gravity * system.Mass, 0.0f));
    //     for (auto const spring : system.Springs) {
    //         auto const p0 = spring.AdjIdx.first;
    //         auto const p1 = spring.AdjIdx.second;
    //         glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
    //         glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
    //         glm::vec3 const e01 = glm::normalize(x01);
    //         glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
    //         forces[p0] += f;
    //         forces[p1] -= f;
    //     }
    //     for (std::size_t i = 0; i < system.Positions.size(); i++) {
    //         if (system.Fixed[i])
    //             continue;
    //         // system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
    //         system.Velocities[i] += (forces[i] / system.Mass) * ddt;
    //         system.Positions[i] += system.Velocities[i] * ddt;
    //     }
    // }

    std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0.0f, -system.Gravity * system.Mass, 0.0f));

    std::vector<Eigen::Triplet<float>> triplets_xk;
    for (std::size_t i = 0; i < system.Positions.size() * 3; i++) {
        triplets_xk.emplace_back(i, i, system.Mass / (dt * dt));
    }
    Eigen::SparseMatrix<float> Hg_xk = CreateEigenSparseMatrix(system.Positions.size() * 3, triplets_xk);

    for (auto const spring : system.Springs) {
        auto const p0 = spring.AdjIdx.first;
        auto const p1 = spring.AdjIdx.second;
        glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
        glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
        glm::vec3 const e01 = glm::normalize(x01);
        // glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
        glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength)) * e01;
        forces[p0] += f;
        forces[p1] -= f;

        glm::mat3 He_ij(
            x01.x * x01.x, x01.x * x01.y, x01.x * x01.z,
            x01.y * x01.x, x01.y * x01.y, x01.y * x01.z,
            x01.z * x01.x, x01.z * x01.y, x01.z * x01.z);

        He_ij /= glm::dot(x01, x01);

        glm::mat3 He_I(
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f);

        glm::mat3 He = system.Stiffness * He_ij + system.Stiffness * (1 - spring.RestLength / glm::length(x01)) * (He_I - He_ij);

        std::vector<Eigen::Triplet<float>> triplets;
        for (std::size_t i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < 3; j++) {
                triplets.emplace_back(p0 * 3 + i, p0 * 3 + j, He[i][j]);
                triplets.emplace_back(p1 * 3 + i, p1 * 3 + j, He[i][j]);
                triplets.emplace_back(p0 * 3 + i, p1 * 3 + j, -He[i][j]);
                triplets.emplace_back(p1 * 3 + i, p0 * 3 + j, -He[i][j]);
            }
        }
        Eigen::SparseMatrix<float> Hg_xk_ij = CreateEigenSparseMatrix(system.Positions.size() * 3, triplets);
        Hg_xk += Hg_xk_ij;
    }

    // get -lambler_g_xk
    Eigen::VectorXf negativ_lambler_g_xk = glm2eigen(system.Velocities) * system.Mass / dt;
    Eigen::VectorXf eigen_forces = glm2eigen(forces);
    negativ_lambler_g_xk = negativ_lambler_g_xk + eigen_forces;

    std::vector<glm::vec3> delta_xk = eigen2glm(ComputeSimplicialLLT(Hg_xk, negativ_lambler_g_xk));

    // get v_k+1
    for (std::size_t i = 0; i < system.Positions.size(); i++) {
        if (system.Fixed[i])
            continue;
        system.Positions[i] += delta_xk[i];
        // system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
    }

    // need to renew forces
    std::vector<glm::vec3> new_forces(system.Positions.size(), glm::vec3(0.0f, -system.Gravity * system.Mass, 0.0f));
    for (auto const spring : system.Springs) {
        auto const p0 = spring.AdjIdx.first;
        auto const p1 = spring.AdjIdx.second;
        glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
        glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
        glm::vec3 const e01 = glm::normalize(x01);
        // glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
        glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength)) * e01;
        new_forces[p0] += f;
        new_forces[p1] -= f;
    }
    for (std::size_t i = 0; i < system.Positions.size(); i++) {
        if (system.Fixed[i])
            continue;
        system.Velocities[i] += (new_forces[i] / system.Mass) * dt;
    }
}

}  // namespace VCX::Labs::Animation
