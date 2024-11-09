package frc.robot.subsystems.rollers.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.subsystems.rollers.GenericRollers;

public class Intake extends GenericRollers<Intake.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(12),
    SHOOT_SPEAKER(0),
    SHOOT_AMP(0),
    SPEAKER_TRANSFER(0),
    AMP_TRANSFER(0),
    EJECT(-8),
    AMP_EJECT(0);

    private int volts;

    private Target(int volts) {
      this.volts = volts;
    }

    public int getVolts() {
      return volts;
    }
  }

  private Target voltageTarget = Target.IDLE;
  private Target lastTarget = Target.IDLE;
  private Debouncer debouncer = new Debouncer(0.2, DebounceType.kFalling); // tune timing

  public Intake(IntakeIO intakeIO) {
    super("Intake", intakeIO);
    setVoltageTarget(Target.IDLE);
  }

  public boolean isContactingNote() {
    return debouncer.calculate(inputs.supplyCurrentAmps > 40) // FIXME tune
        && voltageTarget == Target.INTAKE
        && stateTimer.hasElapsed(0.2); // tune
  }
}
