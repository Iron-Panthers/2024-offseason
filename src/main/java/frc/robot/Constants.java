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
  public static RobotType ROBOT_TYPE = RobotType.DEV;

  public static Mode getRobotMode() {
    return switch (ROBOT_TYPE) {
      case COMP, DEV -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM -> Mode.SIM;
    };
  }

  public static RobotType getRobotType() {
    return ROBOT_TYPE;
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

  public static final class State {
    public static final double POSE_BUFFER_SIZE_SECONDS = 1.5;
  }

  // measured in meters (per sec) & radians (per sec)
  public static final class Swerve {

    public static final DrivebaseConfig DRIVE_CONFIG =
        switch (getRobotType()) {
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
            new ModuleConfig(5, 6, 1, new Rotation2d(2.0 * Math.PI * 0), true, false),
            new ModuleConfig(7, 8, 2, new Rotation2d(2.0 * Math.PI * 0), true, true),
            new ModuleConfig(9, 10, 3, new Rotation2d(2.0 * Math.PI * 0), true, false),
            new ModuleConfig(11, 12, 4, new Rotation2d(2.0 * Math.PI * 0), true, true)
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
              0.4, 0.6, 0, 11, 0, 0.32, 0.11, 0, 3, 0, 5.357142857142857, 21.428571428571427, 3.125);
          case DEV -> new ModuleConstants(
              0.4, 0.6, 0, 11, 0, 0.32, 0.11, 0, 3, 0, 5.357142857142857, 21.428571428571427, 3.125);
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
}
