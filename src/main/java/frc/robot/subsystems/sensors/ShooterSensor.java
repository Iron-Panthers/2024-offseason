package frc.robot.subsystems.sensors;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterSensor implements GenericSensorsIO {
  private final DigitalInput sensor;

  public ShooterSensor() {
    sensor = new DigitalInput(8);


  }

  @Override
  public void updateInputs(GenericSensorIOInputs inputs) {
    inputs.triggered = sensor.get();
  }
  public boolean get(){
    return sensor.get();
  }
}
