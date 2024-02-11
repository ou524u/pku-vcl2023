#pragma once

#include "Labs/4-Animation/MassSpringSystem.h"
// #include "IKSystem.h"

namespace VCX::Labs::Animation {
// lab4 inverse kinematics
// void ForwardKinematics(IKSystem& ik, int StartIndex);
// void InverseKinematicsCCD(IKSystem& ik, const glm::vec3& EndPosition, int maxCCDIKIteration, float eps);
// void InverseKinematicsFABR(IKSystem& ik, const glm::vec3& EndPosition, int maxFABRIKIteration, float eps);

// explixit euler
void BasicMassSpringSystem(MassSpringSystem&, float const);

// lab4 mass spring system, implicit euler
void AdvanceMassSpringSystem(MassSpringSystem&, float const);

// fast
void FastMassSpringSystem(MassSpringSystem&, float const, bool const);

void setgotQ(bool new_gotQ);
}  // namespace VCX::Labs::Animation
