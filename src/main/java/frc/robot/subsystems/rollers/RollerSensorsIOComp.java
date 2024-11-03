package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.concurrent.TimeUnit;

public class RollerSensorsIOComp implements RollerSensorsIO {
  // FIXME; pretty sure rio DIO pullup
  private final DigitalInput serializerSensor = new DigitalInput(-1);
  private final DigitalInput acceleratorSensor = new DigitalInput(-1);
  private final DigitalGlitchFilter noiseFilter = new DigitalGlitchFilter();

  public RollerSensorsIOComp() {
    noiseFilter.setPeriodNanoSeconds(TimeUnit.MILLISECONDS.toNanos(1));
  }

  @Override
  public void updateInputs(RollerSensorsIOInputs inputs) {
    inputs.serializerDetected = !serializerSensor.get();
    inputs.acceleratorDetected = !acceleratorSensor.get();
  }
}
