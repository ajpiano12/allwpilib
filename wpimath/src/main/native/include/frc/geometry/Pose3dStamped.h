// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/geometry/Pose3d.h"

#include "units/time.h"

namespace wpi {
class json;
}  // namespace wpi

namespace frc {

/**
 * Represents a 3D pose containing translational and rotational elements.
 */
class WPILIB_DLLEXPORT Pose3dStamped {
 public:
  /**
   * Constructs a pose at the origin facing toward the positive X axis at time 0
   */
  constexpr Pose3dStamped() = default;

  /**
   * Constructs a pose with a Pose3d and a time value
   */
  Pose3dStamped(Pose3d pose, units::second_t t);

  /**
   * Returns the underlying pose.
   *
   * @return Reference to the pose component of the time stamped pose.
   */
  const Pose3d& Pose() const { return m_pose; }

   /**
   * Returns the underlying time.
   *
   * @return Reference to the time component of the time stamped pose.
   */
  constexpr units::second_t T() const { return m_t; }

 private:
  Pose3d m_pose;
  units::second_t m_t = 0.0_s;
};

WPILIB_DLLEXPORT
void to_json(wpi::json& json, const Pose3dStamped& pose);

WPILIB_DLLEXPORT
void from_json(const wpi::json& json, Pose3dStamped& pose);

}  // namespace frc
