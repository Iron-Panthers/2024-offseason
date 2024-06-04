package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO moduleIO;
  private final int index;

  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO moduleIO, int index) {
    this.moduleIO = moduleIO;
    this.index = index;
  }

  public void updateInputs() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);
  }

  public void runToSetpoint(SwerveModuleState targetState) {}

  public Rotation2d getSteerHeading() {
    return inputs.steerAbsolutePostion;
  }
}
