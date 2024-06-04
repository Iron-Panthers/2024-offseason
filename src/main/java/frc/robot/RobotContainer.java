package frc.robot;

import frc.robot.Constants.Swerve;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIOFalcon500;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Drive swerve =
      new Drive(
          new GyroIOPigeon2(),
          new ModuleIOFalcon500(Swerve.MODULE_CONFIGS[0]),
          new ModuleIOFalcon500(Swerve.MODULE_CONFIGS[1]),
          new ModuleIOFalcon500(Swerve.MODULE_CONFIGS[2]),
          new ModuleIOFalcon500(Swerve.MODULE_CONFIGS[3]));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}
}
