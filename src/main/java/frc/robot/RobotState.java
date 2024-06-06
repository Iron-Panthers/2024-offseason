package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
  private Pose2d estimatedPose;

  private static RobotState instance;

  public static RobotState getInstance() {
    return instance;
  }

  public RobotState() {

  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }
}
