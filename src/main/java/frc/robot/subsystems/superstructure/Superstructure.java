package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class Superstructure extends SubsystemBase {
  public enum TargetState {
    STOW;
  }

  private final Elevator elevator;

  private TargetState targetState = TargetState.STOW;

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {

    targetState = TargetState.STOW;
    switch (targetState) {
      case STOW -> {}
    }
  }

  public TargetState getTargetState() {
    return targetState;
  }

  public void setTargetState(TargetState state) {
    targetState = state;
  }

  public Command setTargetCommand(TargetState state) {
    return this.runOnce(() -> setTargetState(state));
  }
}
