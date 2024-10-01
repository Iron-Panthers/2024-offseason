// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static RobotType ROBOT_TYPE = RobotType.COMP;

  public static Mode getRobotMode() {
    return switch (ROBOT_TYPE) {
      case COMP, DEV -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    COMP,
    DEV,
    SIM;
  }

  // measured in meters (per sec) & radians (per sec)
  public static final class Swerve {

    public static final DrivebaseConfig DRIVE_CONFIG =
        switch (ROBOT_TYPE) {
          case COMP, SIM -> new DrivebaseConfig(
              Units.inchesToMeters(2),
              Units.inchesToMeters(22.5),
              Units.inchesToMeters(38.5),
              Units.inchesToMeters(33),
              5.4764, // FIXME
              6.7759);
          case DEV -> new DrivebaseConfig(
              Units.inchesToMeters(2),
              Units.inchesToMeters(22.5),
              Units.inchesToMeters(38.5),
              Units.inchesToMeters(33),
              5.4764, // FIXME
              6.7759);
        };

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(DRIVE_CONFIG.trackWidth() / 2, DRIVE_CONFIG.trackWidth() / 2),
          new Translation2d(DRIVE_CONFIG.trackWidth() / 2, -DRIVE_CONFIG.trackWidth() / 2),
          new Translation2d(-DRIVE_CONFIG.trackWidth() / 2, DRIVE_CONFIG.trackWidth() / 2),
          new Translation2d(-DRIVE_CONFIG.trackWidth() / 2, -DRIVE_CONFIG.trackWidth() / 2)
        }; // meters relative to center, NWU convention; fl, fr, bl, br

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    public static final int GYRO_ID = 0;

    // fl, fr, bl, br
    public static final ModuleConfig[] MODULE_CONFIGS =
        switch (ROBOT_TYPE) {
          case COMP, SIM -> new ModuleConfig[] {
            new ModuleConfig(5, 6, 1, new Rotation2d(2 * Math.PI * 0)),
            new ModuleConfig(7, 8, 2, new Rotation2d(2 * Math.PI * 0)),
            new ModuleConfig(9, 10, 3, new Rotation2d(2 * Math.PI * 0)),
            new ModuleConfig(11, 12, 4, new Rotation2d(2 * Math.PI * 0))
          };
          case DEV -> new ModuleConfig[] {
            new ModuleConfig(2, 1, 27, new Rotation2d(2 * Math.PI * 0.316650390625)),
            new ModuleConfig(13, 12, 26, new Rotation2d(2 * Math.PI * 0.225341796875)),
            new ModuleConfig(4, 3, 24, new Rotation2d(2 * Math.PI * 0.41943359375)),
            new ModuleConfig(11, 10, 25, new Rotation2d(2 * Math.PI * -0.39990234375))
          };
        };

    public static final ModuleConstants MODULE_CONSTANTS =
        switch (ROBOT_TYPE) {
          case COMP, SIM -> new ModuleConstants(
              0.4, 0.6, 0, 11, 0, 0.32, 0.11, 0, 3, 0, 5.357142857142857, 21.428571428571427);
          case DEV -> new ModuleConstants(
              0.4, 0.6, 0, 11, 0, 0.32, 0.11, 0, 3, 0, 5.357142857142857, 21.428571428571427);
        };

    public record DrivebaseConfig(
        double wheelRadius,
        double trackWidth,
        double bumperWidthX,
        double bumperWidthY,
        double maxLinearVelocity,
        double maxAngularVelocity) {}

    public record ModuleConfig(
        int driveID, int steerID, int encoderID, Rotation2d absoluteEncoderOffset) {}

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
        double steerReduction) {}

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
}
