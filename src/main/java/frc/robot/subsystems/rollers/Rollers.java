package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.accelerator.Accelerator;
import frc.robot.subsystems.rollers.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  public enum RollerState {
    IDLE,
    INTAKE
  }

  private final Intake intake;
  private final Accelerator accelerator;

  private RollerState targetState = RollerState.IDLE;

  public Rollers(Intake intake, Accelerator accelerator) {
    this.intake = intake;
    this.accelerator = accelerator;
  }

  @Override
  public void periodic() {
    intake.setVoltageTarget(Intake.Target.IDLE);
    accelerator.setVoltageTarget(Accelerator.Target.IDLE);
    switch (targetState) {
      case IDLE -> {}
      case INTAKE -> {
        intake.setVoltageTarget(Intake.Target.INTAKE);
        accelerator.setVoltageTarget(Accelerator.Target.INTAKE);
      }
    }

    intake.periodic();
    accelerator.periodic();

    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public RollerState getTargetState() {
    return targetState;
  }

  public void setTargetState(RollerState targetState) {
    this.targetState = targetState;
  }

  public Command setTargetCommand(RollerState target) {
    return startEnd(
        () -> {
          this.targetState = target;
        },
        () -> {
          this.targetState = RollerState.IDLE;
        });
  }
}
