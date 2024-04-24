package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final int index;

  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);
  }
}
