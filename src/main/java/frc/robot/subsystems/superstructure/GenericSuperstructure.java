package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

public class GenericSuperstructure<G extends GenericSuperstructure.PositionTarget> {
  public interface PositionTarget {
    double getPosition();
  }

  private final String name;
  private final GenericSuperstructureIO superstructureIO;
  private GenericSuperstructureIOInputsAutoLogged inputs =
      new GenericSuperstructureIOInputsAutoLogged();
  private G positionTarget;
  private boolean zeroing;

  public GenericSuperstructure(String name, GenericSuperstructureIO superstructureIO) {
    this.name = name;
    this.superstructureIO = superstructureIO;
  }

  public void periodic() {
    superstructureIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    if (zeroing) {
      superstructureIO.runCharacterization();
      if (inputs.velocityRotPerSec < 0.01) {
        zeroing = false;
        superstructureIO.setOffset();
      }
    } else {
      superstructureIO.runPosition(positionTarget.getPosition());
    }

    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + name + "/Target", zeroing);
  }

  public G getGetPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(G positionTarget) {
    this.positionTarget = positionTarget;
  }

  public void runCharacterization() {
    zeroing = true;
  }

  public boolean atPosition() {
    return Math.abs(inputs.positionRotations - positionTarget.getPosition()) < 0.5;
  }
}
