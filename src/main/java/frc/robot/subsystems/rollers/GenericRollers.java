package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public abstract class GenericRollers<G extends GenericRollers.VoltageTarget> {
  public interface VoltageTarget {
    int getVolts();
  }

  private final String name;
  private final GenericRollersIO rollerIO;
  protected GenericRollersIOInputsAutoLogged inputs = new GenericRollersIOInputsAutoLogged();
  protected Timer stateTimer = new Timer();

  private G voltageTarget;
  private G lastTarget;

  public GenericRollers(String name, GenericRollersIO rollerIO) {
    this.name = name;
    this.rollerIO = rollerIO;

    stateTimer.start();
  }

  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    if (voltageTarget != lastTarget) {
      stateTimer.reset();
      lastTarget = voltageTarget;
    }

    rollerIO.runVolts(voltageTarget.getVolts());
    Logger.recordOutput("Rollers/" + name + "/Target", voltageTarget.toString());
  }

  public G getVoltageTarget() {
    return voltageTarget;
  }

  public void setVoltageTarget(G voltageTarget) {
    this.voltageTarget = voltageTarget;
  }
}
