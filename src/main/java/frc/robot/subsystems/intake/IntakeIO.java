package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public boolean connected = true;
    public double positionRads = 0;
    public double velocityRadsPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void stop() {}
}
