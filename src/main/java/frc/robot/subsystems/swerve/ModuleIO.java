package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  class ModuleIOInputs {
    public boolean driveMotorConnected = true;
    public boolean steerMotorConnected = true;

    public double drivePositionRads = 0;
    public double driveVelocityRadsPerSec = 0;
    public double driveAppliedVolts = 0;

    public Rotation2d steerAbsolutePostion = new Rotation2d();
    public Rotation2d steerPosition = new Rotation2d();
    public double steerVelocityRadsPerSec = 0;
    public double steerAppliedVolts = 0;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void runDriveVolts(double volts) {}

  default void runSteerVolts(double volts) {}

  default void runDriveVelocitySetpoint(double velocityRadsPerSec) {}

  default void runSteerPositionSetpoint(double angleRads) {}

  default void setDriveSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void setSteerSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void stop() {}
}
