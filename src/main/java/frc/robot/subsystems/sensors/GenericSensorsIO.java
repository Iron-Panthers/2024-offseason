package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;


public interface GenericSensorsIO {
  @AutoLog
  class GenericSensorIOInputs {
    public boolean triggered = false;
  }

  default void updateInputs(GenericSensorIOInputs inputs) {}
}
