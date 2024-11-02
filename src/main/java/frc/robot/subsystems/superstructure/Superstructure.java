package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    STOW,
    INTAKE,
    TEST,
    AMP,
    ZERO
  }

  private SuperstructureState targetState = SuperstructureState.STOW;

  private final Elevator elevator;

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    switch (targetState) {
      case STOW -> {
        elevator.setTarget(ElevatorTarget.STOW);
      }
      case INTAKE -> {
        elevator.setTarget(ElevatorTarget.STOW);
      }
      case TEST -> {
        elevator.setTarget(ElevatorTarget.STOW);
      }
      case AMP -> {
        elevator.setTarget(ElevatorTarget.AMP);
      }
      case ZERO -> {
        elevator.runZero();
      }
    }
    elevator.periodic();
    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public void setTargetState(SuperstructureState superstructureState) {
    targetState = superstructureState;
  }
}
