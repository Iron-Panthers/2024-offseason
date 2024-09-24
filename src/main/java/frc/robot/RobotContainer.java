// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOFalcon500;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Drive swerve; // FIXME make final, implement other robot types

  public RobotContainer() {
    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.ROBOT_TYPE) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOFalcon500(Swerve.MODULE_CONFIGS[0]),
                  new ModuleIOFalcon500(Swerve.MODULE_CONFIGS[1]),
                  new ModuleIOFalcon500(Swerve.MODULE_CONFIGS[2]),
                  new ModuleIOFalcon500(Swerve.MODULE_CONFIGS[3]));
        }
        case DEV -> {}
        case SIM -> {}
      }
    }

    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(), -driverA.getLeftX(), -driverA.getRightX());
                })
            .withName("Drive Teleop"));
  }

  private void configureAutos() {}
}
