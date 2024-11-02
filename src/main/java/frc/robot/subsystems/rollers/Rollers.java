package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.accelerator.Accelerator;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.serializer.Serializer;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  public enum RollerState {
    IDLE,
    INTAKE,
    SHOOT_SPEAKER,
    SHOOT_AMP,
    SPEAKER_TRANSFER,
    AMP_TRANSFER,
    EJECT
  }

  private final Intake intake;
  private final Accelerator accelerator;
  private final Serializer serializer;
  private final GenericRollers[] genericRollers;

  private RollerState targetState = RollerState.IDLE;

  public Rollers(Intake intake, Accelerator accelerator, Serializer serializer) {
    this.intake = intake;
    this.accelerator = accelerator;
    this.serializer = serializer;
    genericRollers = new GenericRollers[3];
    genericRollers[0] = intake;
    genericRollers[1] = accelerator;
    genericRollers[2] = serializer;
  }

  @Override
  public void periodic() {
    for (GenericRollers g : genericRollers) {
      g.setVoltageTarget(RollerState.IDLE);
      g.setVoltageTarget(targetState);
    }
    for (GenericRollers g : genericRollers) {
      g.periodic();
    }

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
