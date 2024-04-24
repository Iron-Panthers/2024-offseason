package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveConstants {

  public record ModuleConfig(
      int driveID, int steerID, int encoderID, Rotation2d absoluteEncoderOffset) {}

  public record ModuleConstants() {}
}
