#include "Labs/4-Animation/tasks.h"
#include <spdlog/spdlog.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
// #include "CustomFunc.inl"
// #include "IKSystem.h"

namespace VCX::Labs::Animation {

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

static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::size_t m, std::vector<Eigen::Triplet<float>> const& triplets) {
    Eigen::SparseMatrix<float> matLinearized(n, m);
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

void BasicMassSpringSystem(MassSpringSystem& system, float const dt) {
    int const steps = 1000;
    float const ddt = dt / steps;
    for (std::size_t s = 0; s < steps; s++) {
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0.0f, -system.Gravity * system.Mass, 0.0f));
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
            glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
            forces[p0] += f;
            forces[p1] -= f;
        }
        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            if (system.Fixed[i])
                continue;
            // system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
            system.Velocities[i] += (forces[i] / system.Mass) * ddt;
            system.Positions[i] += system.Velocities[i] * ddt;
        }
    }
}

void AdvanceMassSpringSystem(MassSpringSystem& system, float const dt) {
    // your code here: rewrite following code

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

// Fast Mass-Spring below.

#include <Eigen/Cholesky>

// Solve Ax = b using Cholesky decomposition
static Eigen::VectorXf ComputeCholesky(
    Eigen::SparseMatrix<float> const& A,
    Eigen::VectorXf const& b) {
    // Perform Cholesky decomposition
    Eigen::SimplicialLLT<Eigen::SparseMatrix<float>> choleskySolver(A);

    if (choleskySolver.info() != Eigen::Success) {
        // Cholesky decomposition failed
        // Handle the error or return an appropriate value
        // For example, throw an exception
        throw std::runtime_error("Cholesky decomposition failed");
    }

    // Solve the system using the Cholesky factorization
    return choleskySolver.solve(b);
}

bool gotQ = false;
Eigen::SparseMatrix<float> precalc_Q;
Eigen::SparseMatrix<float> precalc_J;

void calc_Q(MassSpringSystem& system, float const dt) {
    // sparce matrix init
    std::vector<Eigen::Triplet<float>> triplets_Q;
    for (std::size_t i = 0; i < system.Positions.size() * 3; i++) {
        triplets_Q.emplace_back(i, i, 1.0f);
    }
    Eigen::SparseMatrix<float> h2_Q = CreateEigenSparseMatrix(system.Positions.size() * 3, triplets_Q);

    Eigen::SparseMatrix<float> h2_J(system.Positions.size() * 3, system.Springs.size() * 3);

    for (int spr_seq = 0; spr_seq < system.Springs.size(); spr_seq++) {
        auto const spring = system.Springs[spr_seq];
        auto const p0 = spring.AdjIdx.first;
        auto const p1 = spring.AdjIdx.second;
        glm::mat3 He_I(
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f);
        glm::mat3 He_Lij = (dt * dt) * system.Stiffness * (1.0f / system.Mass) * He_I;

        std::vector<Eigen::Triplet<float>> triplets_Lij;
        std::vector<Eigen::Triplet<float>> triplets_Jij;

        for (std::size_t i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < 3; j++) {
                triplets_Lij.emplace_back(p0 * 3 + i, p0 * 3 + j, He_Lij[i][j]);
                triplets_Lij.emplace_back(p1 * 3 + i, p1 * 3 + j, He_Lij[i][j]);
                triplets_Lij.emplace_back(p0 * 3 + i, p1 * 3 + j, -He_Lij[i][j]);
                triplets_Lij.emplace_back(p1 * 3 + i, p0 * 3 + j, -He_Lij[i][j]);

                triplets_Jij.emplace_back(p0 * 3 + i, spr_seq * 3 + j, He_Lij[i][j]);
                triplets_Jij.emplace_back(p1 * 3 + i, spr_seq * 3 + j, -He_Lij[i][j]);
            }
        }
        Eigen::SparseMatrix<float> h2_L_ij = CreateEigenSparseMatrix(system.Positions.size() * 3, triplets_Lij);
        Eigen::SparseMatrix<float> h2_J_ij = CreateEigenSparseMatrix(system.Positions.size() * 3, system.Springs.size() * 3, triplets_Jij);

        h2_Q += h2_L_ij;
        h2_J += h2_J_ij;
    }

    precalc_Q = h2_Q;
    precalc_J = h2_J;
    return;
}

static Eigen::VectorXf mat2eigen(Eigen::SparseMatrix<float> const& mat_v) {
    // Convert the sparse matrix into a dense matrix
    Eigen::MatrixXf denseMat = mat_v;

    // Reshape the dense matrix into a vector
    Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf>(denseMat.data(), denseMat.size());

    return v;
}
void setgotQ(bool new_gotQ) {
    gotQ = new_gotQ;
    return;
}
void FastMassSpringSystem(MassSpringSystem& system, float const dt, bool const check_collision) {
    if (gotQ == false) {
        calc_Q(system, dt);
        gotQ = true;
    }
    // local slover
    // d is [3*s, 1]
    std::vector<Eigen::Triplet<float>> triplets_d;
    for (int spr_seq = 0; spr_seq < system.Springs.size(); spr_seq++) {
        auto const spring = system.Springs[spr_seq];
        auto const p0 = spring.AdjIdx.first;
        auto const p1 = spring.AdjIdx.second;

        glm::vec3 const x01 = system.Positions[p0] - system.Positions[p1];
        triplets_d.emplace_back(spr_seq * 3, 0, x01.x / glm::length(x01) * spring.RestLength);
        triplets_d.emplace_back(spr_seq * 3 + 1, 0, x01.y / glm::length(x01) * spring.RestLength);
        triplets_d.emplace_back(spr_seq * 3 + 2, 0, x01.z / glm::length(x01) * spring.RestLength);
    }

    Eigen::SparseMatrix<float> mat_d = CreateEigenSparseMatrix(system.Springs.size() * 3, 1, triplets_d);
    Eigen::SparseMatrix<float> mat_Jd = precalc_J * mat_d;
    Eigen::VectorXf b = mat2eigen(mat_Jd);

    Eigen::VectorXf x_curr = glm2eigen(system.Positions);
    Eigen::VectorXf hv_curr = glm2eigen(system.Velocities) * dt;
    std::vector<glm::vec3> vecf_ext(system.Positions.size(), glm::vec3(0.0f, -system.Gravity, 0.0f));
    Eigen::VectorXf f_ext = glm2eigen(vecf_ext) * (2 * dt * dt);

    b = b + x_curr + hv_curr + f_ext;

    // slove!
    std::vector<glm::vec3> new_xk = eigen2glm(ComputeCholesky(precalc_Q, b));
    std::vector<bool> collision(system.Positions.size(), false);

    // upd x
    for (std::size_t i = 0; i < system.Positions.size(); i++) {
        if (system.Fixed[i])
            continue;

        if (check_collision) {
            if (glm::length(new_xk[i]) < 1.0f) {
                // do not change position
                collision[i] = true;
            } else {
                system.Positions[i] = new_xk[i];
            }
        } else {
            system.Positions[i] = new_xk[i];
        }
    }
    // we need new velocities as well
    // need to renew forces first
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
        if (check_collision) {
            if (collision[i]) {
                system.Velocities[i] = 2.0f * system.Velocities[i] * glm::normalize(system.Positions[i]) - system.Velocities[i];
                system.Velocities[i] *= 0.9;
            } else {
                system.Velocities[i] += (new_forces[i] / system.Mass) * dt;
            }

        } else {
            system.Velocities[i] += (new_forces[i] / system.Mass) * dt;
        }
    }
    return;
}

}  // namespace VCX::Labs::Animation
