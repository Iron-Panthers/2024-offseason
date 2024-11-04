package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.Intake.Target;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  public enum RollerState {
    IDLE,
    INTAKE
  }

  private final Intake intake;

  private final RollerSensorsIO sensorsIO;
  private RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

  private RollerState targetState = RollerState.IDLE;

  public Rollers(Intake intake, RollerSensorsIO sensorsIO) {
    this.intake = intake;
    this.sensorsIO = sensorsIO;
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorsInputs);
    Logger.processInputs("RollerSensors", sensorsInputs);

    intake.setVoltageTarget(Target.IDLE);

    switch (targetState) {
      case IDLE -> {}
      case INTAKE -> {
        intake.setVoltageTarget(Target.INTAKE);
      }
    }

    intake.periodic();

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

  @AutoLogOutput
  public boolean isContactingNote() {
    return intake.isContactingNote();
  }
}
