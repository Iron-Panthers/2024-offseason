package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollers;
import frc.robot.subsystems.rollers.Rollers;

public class Intake extends GenericRollers<Intake.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(12),
    SHOOT_SPEAKER(0),
    SHOOT_AMP(0),
    SPEAKER_TRANSFER(0),
    AMP_TRANSFER(0),
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

  public Intake(IntakeIO intakeIO) {
    super("Intake", intakeIO);
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
