// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
  private Pose2d estimatedPose;

  private static RobotState instance;

  public static RobotState getInstance() {
    return instance;
  }

  public RobotState() {}

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }
}
