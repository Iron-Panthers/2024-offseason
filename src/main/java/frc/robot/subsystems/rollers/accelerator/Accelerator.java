package frc.robot.subsystems.rollers.accelerator;

import frc.robot.subsystems.rollers.GenericRollers;
import frc.robot.subsystems.rollers.Rollers;

public class Accelerator extends GenericRollers<Accelerator.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(0),
    SHOOT_SPEAKER(12),
    SHOOT_AMP(0),
    SPEAKER_TRANSFER(5),
    AMP_TRANSFER(-5),
    EJECT(0);

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

  @Override
  public void setVoltageTarget(Rollers.RollerState rVoltageState) {
    switch (rVoltageState) {
      case IDLE -> {
        voltageTarget = Target.IDLE;
      }
      case INTAKE -> {
        voltageTarget = Target.INTAKE;
      }
      case SHOOT_SPEAKER -> {
        voltageTarget = Target.SHOOT_SPEAKER;
      }
      case SHOOT_AMP -> {
        voltageTarget = Target.SHOOT_AMP;
      }
      case SPEAKER_TRANSFER -> {
        voltageTarget = Target.SPEAKER_TRANSFER;
      }
      case AMP_TRANSFER -> {
        voltageTarget = Target.AMP_TRANSFER;
      }
      case EJECT -> {
        voltageTarget = Target.EJECT;
      }
    }
  }
}
