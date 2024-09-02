package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.State;

public class RobotState {
  public record OdometryMeasurement(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionMeasurement(Pose2d visionPose, double timestamp) {}

  private TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(State.POSE_BUFFER_SIZE_SECONDS);

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });
  private Rotation2d lastGyroAngle = new Rotation2d();

  private static RobotState instance;

  public static RobotState getInstance() {
    return instance;
  }

  private RobotState() {}

  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    Twist2d twist =
        Constants.Swerve.KINEMATICS.toTwist2d(lastWheelPositions, measurement.wheelPositions());
    lastWheelPositions = measurement.wheelPositions();

    twist.dtheta = measurement.gyroAngle().minus(lastGyroAngle).getRadians();
    lastGyroAngle = measurement.gyroAngle();

    odometryPose = odometryPose.exp(twist);

    poseBuffer.addSample(measurement.timestamp(), odometryPose);
  }

  public void addVisionMeasurement(VisionMeasurement measurement) {
    // if measurement is old enough to be outside buffer timespan, skip
    if (poseBuffer.getInternalBuffer().isEmpty()
        || poseBuffer.getInternalBuffer().lastKey() < State.POSE_BUFFER_SIZE_SECONDS) {
      return;
    }
  }

  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }
}
