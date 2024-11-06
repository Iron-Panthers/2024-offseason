package frc.robot.subsystems.rollers.accelerator;

import frc.robot.subsystems.rollers.GenericRollers;

public class Accelerator extends GenericRollers<Accelerator.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(2),
    SHOOT_SPEAKER(12),
    SHOOT_AMP(0),
    SPEAKER_TRANSFER(2),
    AMP_TRANSFER(-3),
    EJECT(0);

    private int volts;

    private Target(int volts) {
      this.volts = volts;
    }

    public int getVolts() {
      return volts;
    }
  }

  public Accelerator(AcceleratorIO acceleratorIO) {
    super("Accelerator", acceleratorIO);
    setVoltageTarget(Target.IDLE);
  }
}
