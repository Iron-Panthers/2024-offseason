// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Swerve {
    public static final class Measurements {
      public static final double TRACKWIDTH_METERS = 0.5207; // square drivebase

      public static final Translation2d[] MODULE_TRANSLATIONS =
          new Translation2d[] {
            new Translation2d(TRACKWIDTH_METERS / 2, TRACKWIDTH_METERS / 2),
            new Translation2d(TRACKWIDTH_METERS / 2, -TRACKWIDTH_METERS / 2),
            new Translation2d(-TRACKWIDTH_METERS / 2, TRACKWIDTH_METERS / 2),
            new Translation2d(-TRACKWIDTH_METERS / 2, -TRACKWIDTH_METERS / 2)
          }; // meters relative to center, NWU convention; fl, fr, bl, br

      public static final double MAX_VELOCITY_MPS = 6380.0 / 60.0 * 0.10033 * (1 / 6.12) * Math.PI;
    }

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(Measurements.MODULE_TRANSLATIONS);

    // fl, fr, bl, br
    public static final ModuleConfig[] MODULE_CONFIGS =
        new ModuleConfig[] {
          new ModuleConfig(2, 1, 27, new Rotation2d(2 * Math.PI * 0.316650390625)),
          new ModuleConfig(13, 12, 26, new Rotation2d(2 * Math.PI * 0.225341796875)),
          new ModuleConfig(4, 3, 24, new Rotation2d(2 * Math.PI * 0.41943359375)),
          new ModuleConfig(11, 10, 25, new Rotation2d(2 * Math.PI * -0.39990234375))
        };

    public record ModuleConfig(
        int driveID, int steerID, int encoderID, Rotation2d absoluteEncoderOffset) {}

    public record ModuleConstants() {}
  }
}
