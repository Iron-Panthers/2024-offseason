package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    STOW,
    INTAKE,
    SUBWOOF_SHOT,
    SHUTTLE,
    AMP,
    ZERO
  }

  private SuperstructureState targetState = SuperstructureState.STOW;

  private final Elevator elevator;
  private final Pivot pivot;

  public Superstructure(Elevator elevator, Pivot pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  public void periodic() {
    switch (targetState) {
      case STOW -> {
        pivot.setTarget(PivotTarget.STOW);
        elevator.setTarget(ElevatorTarget.STOW);
      }
      case INTAKE -> {
        pivot.setTarget(PivotTarget.STOW);
        elevator.setTarget(ElevatorTarget.STOW);
      }
      case SUBWOOF_SHOT -> {
        pivot.setTarget(PivotTarget.SUBWOOF_SHOT);
        elevator.setTarget(ElevatorTarget.STOW);
      }
      case SHUTTLE -> {
        pivot.setTarget(PivotTarget.SHUTTLE);
        elevator.setTarget(ElevatorTarget.STOW);
      }
      case AMP -> {
        elevator.setTarget(ElevatorTarget.AMP);
        pivot.setTarget(PivotTarget.STOW);
      }
      case ZERO -> {
        elevator.runZero();
        pivot.runZero();
      }
    }
    elevator.periodic();
    pivot.periodic();
    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public void setTargetState(SuperstructureState superstructureState) {
    targetState = superstructureState;
  }
}
