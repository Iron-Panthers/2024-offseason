package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollers;

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

  public Intake(IntakeIO intakeIO) {
    super("Intake", intakeIO);
    setVoltageTarget(Target.IDLE);
  }
}
