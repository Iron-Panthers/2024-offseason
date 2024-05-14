package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class SwerveConstants {
  public static final class Dimensions {
    public static final double TRACKWIDTH_METERS = 0.5207; // square drivebase

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(TRACKWIDTH_METERS / 2, TRACKWIDTH_METERS / 2),
          new Translation2d(TRACKWIDTH_METERS / 2, -TRACKWIDTH_METERS / 2),
          new Translation2d(-TRACKWIDTH_METERS / 2, TRACKWIDTH_METERS / 2),
          new Translation2d(-TRACKWIDTH_METERS / 2, -TRACKWIDTH_METERS / 2)
        }; // meters relative to center, NWU convention; fl, fr, bl, br
  }

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(Dimensions.MODULE_TRANSLATIONS);

  public record ModuleConfig(
      int driveID, int steerID, int encoderID, Rotation2d absoluteEncoderOffset) {}

  public record ModuleConstants() {}
}
