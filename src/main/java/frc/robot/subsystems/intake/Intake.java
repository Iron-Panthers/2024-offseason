package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum VoltageTarget {
    IDLE(0),
    INTAKE(8), // FIXME
    EJECT(-8);

    private int volts;

    private VoltageTarget(int volts) {
      this.volts = volts;
    }
  }

  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  private VoltageTarget voltageTarget = VoltageTarget.IDLE;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Mechanism/Intake", intakeInputs);

    intakeIO.runVolts(voltageTarget.volts);
  }

  public void setVoltageTarget(VoltageTarget target) {
    voltageTarget = target;
  }
}
