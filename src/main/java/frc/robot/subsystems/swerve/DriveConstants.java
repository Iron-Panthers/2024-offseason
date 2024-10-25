package frc.robot.subsystems.swerve;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // measures in meters (per sec) and radians (per sec)
  public static final DrivebaseConfig DRIVE_CONFIG =
      switch (getRobotType()) {
        case COMP, SIM -> new DrivebaseConfig(
            Units.inchesToMeters(2),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(38.5),
            Units.inchesToMeters(33),
            5, // FIXME
            5);
        case DEV -> new DrivebaseConfig(
            Units.inchesToMeters(2),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(38.5),
            Units.inchesToMeters(33),
            // 5.4764, // FIXME
            // 6.7759);
            4,
            4);
      };

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(DRIVE_CONFIG.trackWidth() / 2.0, DRIVE_CONFIG.trackWidth() / 2.0),
        new Translation2d(DRIVE_CONFIG.trackWidth() / 2.0, -DRIVE_CONFIG.trackWidth() / 2.0),
        new Translation2d(-DRIVE_CONFIG.trackWidth() / 2.0, DRIVE_CONFIG.trackWidth() / 2.0),
        new Translation2d(-DRIVE_CONFIG.trackWidth() / 2.0, -DRIVE_CONFIG.trackWidth() / 2.0)
      }; // meters relative to center, NWU convention; fl, fr, bl, br

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  public static final int GYRO_ID = 0;

  // fl, fr, bl, br
  public static final ModuleConfig[] MODULE_CONFIGS =
      switch (getRobotType()) {
        case COMP -> new ModuleConfig[] {
          new ModuleConfig(5, 6, 1, new Rotation2d(0), true, false),
          new ModuleConfig(7, 8, 2, new Rotation2d(0), true, true),
          new ModuleConfig(9, 10, 3, new Rotation2d(0), true, false),
          new ModuleConfig(11, 12, 4, new Rotation2d(0), true, true)
        };
        case DEV -> new ModuleConfig[] {
          new ModuleConfig(2, 1, 27, new Rotation2d(1.954), true, false),
          new ModuleConfig(13, 12, 26, new Rotation2d(1.465), true, true),
          new ModuleConfig(4, 3, 24, new Rotation2d(2.612), true, false),
          new ModuleConfig(11, 10, 25, new Rotation2d(-2.563), true, true)
        };
        case SIM -> new ModuleConfig[] {
          new ModuleConfig(0, 0, 0, new Rotation2d(0), true, false),
          new ModuleConfig(0, 0, 0, new Rotation2d(0), true, true),
          new ModuleConfig(0, 0, 0, new Rotation2d(0), true, false),
          new ModuleConfig(0, 0, 0, new Rotation2d(0), true, true)
        };
      };

  public static final ModuleConstants MODULE_CONSTANTS =
      switch (getRobotType()) {
        case COMP, SIM -> new ModuleConstants(
            0, // steerkS
            0, // steerkV
            0, // steerkA
            0, // steerkP
            0, // steerkD
            0, // drivekS
            0, // drivekV
            0, // drivekA
            0, // drivekP
            0, // drivekD
            5.357142857142857,
            21.428571428571427,
            3.125);
        case DEV -> new ModuleConstants(
            0, // steerkS
            0, // steerkV
            0, // steerkA
            11, // steerkP
            0, // steerkD
            0, // drivekS
            0, // drivekV
            0, // drivekA
            1.5, // drivekP
            0, // drivekD
            5.357142857142857,
            21.428571428571427,
            3.125);
      };

  public record DrivebaseConfig(
      double wheelRadius,
      double trackWidth,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxAngularVelocity) {}

  public record ModuleConfig(
      int driveID,
      int steerID,
      int encoderID,
      Rotation2d absoluteEncoderOffset,
      boolean steerInverted,
      boolean driveInverted) {}

  public record ModuleConstants(
      double steerkS,
      double steerkV,
      double steerkA,
      double steerkP,
      double steerkD,
      double drivekS,
      double drivekV,
      double drivekA,
      double drivekP,
      double drivekD,
      double driveReduction,
      double steerReduction,
      double couplingGearReduction) {}

  public record TrajectoryFollowerConstants() {}

  private enum Mk4iReductions {
    MK4I_L3((50 / 14) * (16 / 28) * (45 / 15)),
    STEER(150 / 7);

    double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
