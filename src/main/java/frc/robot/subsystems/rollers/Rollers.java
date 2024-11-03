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

  private RollerState targetState = RollerState.IDLE;

  public Rollers(Intake intake, Accelerator accelerator, Serializer serializer) {
    this.intake = intake;
    this.accelerator = accelerator;
    this.serializer = serializer;
  }

  @Override
  public void periodic() {
    switch (targetState) {
      case IDLE -> {
        intake.setVoltageTarget(Intake.Target.IDLE);
        accelerator.setVoltageTarget(Accelerator.Target.IDLE);
        serializer.setVoltageTarget(Serializer.Target.IDLE);
      }
      case INTAKE -> {
        intake.setVoltageTarget(Intake.Target.INTAKE);
        accelerator.setVoltageTarget(Accelerator.Target.INTAKE);
        serializer.setVoltageTarget(Serializer.Target.INTAKE);
      }
      case SHOOT_AMP -> {
        intake.setVoltageTarget(Intake.Target.SHOOT_AMP);
        accelerator.setVoltageTarget(Accelerator.Target.SHOOT_AMP);
        serializer.setVoltageTarget(Serializer.Target.SHOOT_AMP);
      }
      case SHOOT_SPEAKER -> {
        intake.setVoltageTarget(Intake.Target.SHOOT_SPEAKER);
        accelerator.setVoltageTarget(Accelerator.Target.SHOOT_SPEAKER);
        serializer.setVoltageTarget(Serializer.Target.SHOOT_SPEAKER);
      }
      case SPEAKER_TRANSFER -> {
        intake.setVoltageTarget(Intake.Target.SPEAKER_TRANSFER);
        accelerator.setVoltageTarget(Accelerator.Target.SPEAKER_TRANSFER);
        serializer.setVoltageTarget(Serializer.Target.SPEAKER_TRANSFER);
      }
      case AMP_TRANSFER -> {
        intake.setVoltageTarget(Intake.Target.AMP_TRANSFER);
        accelerator.setVoltageTarget(Accelerator.Target.AMP_TRANSFER);
        serializer.setVoltageTarget(Serializer.Target.AMP_TRANSFER);
      }
      case EJECT -> {
        intake.setVoltageTarget(Intake.Target.EJECT);
        accelerator.setVoltageTarget(Accelerator.Target.EJECT);
        serializer.setVoltageTarget(Serializer.Target.EJECT);
      }
    }
    intake.periodic();
    accelerator.periodic();
    serializer.periodic();

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
