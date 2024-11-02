package frc.robot.subsystems.superstructure.pivot;

import org.littletonrobotics.junction.Logger;

public class Pivot {
  public enum PivotTarget {
    STOW(0),
    ZERO(0),
    AMP(32);
    private int position;

    private PivotTarget(int position) {
      this.position = position;
    }

    public int getPosition() {
      return position;
    }
  }

  private final PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PivotTarget target = PivotTarget.STOW;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Pivot", inputs);
    if (target.equals(PivotTarget.ZERO)) {
      runZero();
    } else {
      io.runPosition(target.getPosition());
    }
  }

  public PivotTarget getTarget() {
    return target;
  }

  public void setTarget(PivotTarget target) {
    this.target = target;
  }

  // bad FIXME
  public boolean runZero() {
    io.runCharacterization(PivotConstants.ZEROING_VOLTS); // FIXME

    if (inputs.supplyCurrentAmps >= PivotConstants.ZEROING_CURRENT_LIMIT) {
      io.stop();
      return true;
    }

    return false;
  }
}
