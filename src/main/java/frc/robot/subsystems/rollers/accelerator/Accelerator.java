package frc.robot.subsystems.rollers.accelerator;

import frc.robot.subsystems.rollers.GenericRollers;

public class Accelerator extends GenericRollers<Accelerator.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(12),
    EJECT(-8);

    private int volts;

    private Target(int volts) {
      this.volts = volts;
    }

    public int getVolts() {
      return volts;
    }
  }

  private Target voltageTarget = Target.IDLE;

  public Accelerator(AcceleratorIO acceleratorIO) {
    super("Accelerator", acceleratorIO);
  }
}
