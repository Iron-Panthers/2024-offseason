package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class SerializerSensor implements GenericSensorsIO {
  private final DigitalInput sensor;

  public SerializerSensor() {
    sensor = new DigitalInput(9);
  }

  @Override
  public void updateInputs(GenericSensorIOInputs inputs) {
    inputs.triggered = sensor.get();
  }

  public boolean get() {
    return sensor.get();
  }
}
