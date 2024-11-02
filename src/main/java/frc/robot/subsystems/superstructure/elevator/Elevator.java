package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.Logger;

public class Elevator {
  public enum ElevatorTarget {
    STOW(0),
    ZERO(0),
    AMP(32);
    private int position;

    private ElevatorTarget(int position) {
      this.position = position;
    }

    public int getPosition() {
      return position;
    }
  }

  private final ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ElevatorTarget target = ElevatorTarget.STOW;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Elevator", inputs);
    if (target.equals(ElevatorTarget.ZERO)) {
      runZero();
    } else {
      io.runPosition(target.getPosition());
    }
  }

  public ElevatorTarget getTarget() {
    return target;
  }

  public void setTarget(ElevatorTarget target) {
    this.target = target;
  }

  // bad FIXME
  public boolean runZero() {
    io.runCharacterization(ElevatorConstants.ZEROING_VOLTS); // FIXME

    if (inputs.supplyCurrentAmps >= ElevatorConstants.ZEROING_CURRENT_LIMIT) {
      io.stop();
      return true;
    }

    return false;
  }
}
