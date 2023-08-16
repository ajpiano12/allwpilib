// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/Pose3dStamped.h"

#include <wpi/json.h>

#include "units/time.h"

using namespace frc;

Pose3dStamped::Pose3dStamped(Pose3d pose, units::second_t t)
    : m_pose(std::move(pose)), m_t(std::move(t)) {}

void frc::to_json(wpi::json& json, const Pose3dStamped& pose) {
  json = wpi::json{{"pose", pose.Pose()},
                   {"time", pose.T()}};
}

void frc::from_json(const wpi::json& json, Pose3dStamped& pose) {
  pose = Pose3dStamped{json.at("pose").get<Pose3d>(),
                units::second_t{json.at("time").get<double>()}};
}