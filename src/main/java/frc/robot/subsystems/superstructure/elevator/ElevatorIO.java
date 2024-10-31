package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean connected = true;
    public double positionRotations = 0;
    public double velocityRotPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
    public double tempCelsius = 0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runPosition(double rots) {}

  default void runCharacterization(double volts) {}

  default void stop() {}
}
