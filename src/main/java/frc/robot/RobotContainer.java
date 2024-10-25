// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.VoltageTarget;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;

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
  private Intake intake;
  private Flywheels flywheels;

  public RobotContainer() {
    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(Swerve.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(Swerve.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(Swerve.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(Swerve.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
          flywheels = new Flywheels(new FlywheelsIOTalonFX());
        }
        case DEV -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(Swerve.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(Swerve.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(Swerve.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(Swerve.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
        }
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
    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(), -driverA.getLeftX(), -driverA.getRightX());
                })
            .withName("Drive Teleop"));

    // -----Intake Controls-----
    // FIXME
    driverB
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  intake.setVoltageTarget(VoltageTarget.INTAKE);
                },
                intake));
    driverB
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  intake.setVoltageTarget(VoltageTarget.IDLE);
                },
                intake));
    driverB
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  intake.setVoltageTarget(VoltageTarget.EJECT);
                },
                intake));
  }

  private void configureAutos() {}
}
