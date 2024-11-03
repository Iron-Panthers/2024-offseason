package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface GenericSensorsIO {
  @AutoLog
  class GenericSensorIOInputs {
    public boolean triggered = false;
  }

  default void updateInputs(GenericSensorIOInputs inputs) {}
}
