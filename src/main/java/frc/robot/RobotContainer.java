// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.Flywheels.VelocityTarget;
import frc.robot.subsystems.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.rollers.accelerator.Accelerator;
import frc.robot.subsystems.rollers.accelerator.AcceleratorIOTalonFX;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotIOTalonFX;
import frc.robot.subsystems.rollers.serializer.Serializer;
import frc.robot.subsystems.rollers.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.sensors.SerializerSensor;
import frc.robot.subsystems.sensors.ShooterSensor;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
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
  private Rollers rollers;
  private Flywheels flywheels;
  private Superstructure superstructure;
  private SerializerSensor serializerSensor;
  private ShooterSensor shooterSensor;

  public RobotContainer() {
    Intake intake = null;
    Accelerator accelerator = null;
    Serializer serializer = null;

    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
          accelerator = new Accelerator(new AcceleratorIOTalonFX());
          flywheels = new Flywheels(new FlywheelsIOTalonFX());
          serializer = new Serializer(new SerializerIOTalonFX());
        }
        case DEV -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX()); // FIXME
          accelerator = new Accelerator(new AcceleratorIOTalonFX());
          flywheels = new Flywheels(new FlywheelsIOTalonFX());
          serializer = new Serializer(new SerializerIOTalonFX());
        }
        case SIM -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX()); // FIXME
          accelerator = new Accelerator(new AcceleratorIOTalonFX());
          flywheels = new Flywheels(new FlywheelsIOTalonFX());
          serializer = new Serializer(new SerializerIOTalonFX());
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

    rollers = new Rollers(intake);
    superstructure =
        new Superstructure(new Elevator(new ElevatorIOTalonFX()), new Pivot(new PivotIOTalonFX()));
    serializerSensor = new SerializerSensor();
    shooterSensor = new ShooterSensor();

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
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      -driverA.getRightX(),
                      driverA.getLeftTriggerAxis(),
                      -driverA.getRightTriggerAxis());
                })
            .withName("Drive Teleop"));

    // -----Intake Controls-----
    // intake note and then outtake for a little time
    driverA
        .leftBumper()
        .onTrue(
            new FunctionalCommand(
                    () -> serializerSensor.get(), // because null did not work
                    () -> rollers.setTargetState(RollerState.INTAKE),
                    interrupted -> rollers.setTargetState(RollerState.IDLE),
                    () -> serializerSensor.get(),
                    rollers)
                .andThen(
                    new InstantCommand(() -> rollers.setTargetState(RollerState.EJECT), rollers)
                        .withTimeout(0.6))
                .andThen(new InstantCommand(() -> rollers.setTargetState(RollerState.IDLE))));

    // transfer note to shooter
    driverA
        .b()
        .onTrue(
            new FunctionalCommand(
                () -> serializerSensor.get(), // because null did not work
                () -> rollers.setTargetState(RollerState.SPEAKER_TRANSFER),
                interrupted -> rollers.setTargetState(RollerState.IDLE),
                () -> !shooterSensor.get(),
                rollers));
    // Outtake a little to amp
    driverA
        .x()
        .onTrue(
            new FunctionalCommand(
                    () -> serializerSensor.get(), // because null did not work
                    () -> rollers.setTargetState(RollerState.AMP_TRANSFER),
                    interrupted -> rollers.setTargetState(RollerState.IDLE),
                    () -> serializerSensor.get(),
                    rollers)
                .andThen(
                    new FunctionalCommand(
                        () -> serializerSensor.get(), // because null did not work
                        () -> rollers.setTargetState(RollerState.AMP_TRANSFER),
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> !serializerSensor.get(),
                        rollers)));

    // -----Flywheel Controls-----
    //

    driverA
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheels.setVelocityTarget(VelocityTarget.SLOW);
                },
                flywheels));
    driverA
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheels.setVelocityTarget(VelocityTarget.SHOOT);
                },
                flywheels));
    driverA
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheels.setVelocityTarget(VelocityTarget.IDLE);
                },
                flywheels));
    driverA
        .povDown()
        .onTrue(
            new FunctionalCommand(
                    () -> rollers.setTargetState(Rollers.RollerState.SHOOT_SPEAKER), rollers)
                .alongWith(
                    new InstantCommand(
                        () -> flywheels.setVelocityTarget(Flywheels.VelocityTarget.SHOOT)))
                .andThen(new WaitCommand(2))
                .andThen(
                    new InstantCommand(
                        () -> rollers.setTargetState(Rollers.RollerState.IDLE), rollers))
                .alongWith(
                    new InstantCommand(
                        () -> flywheels.setVelocityTarget(Flywheels.VelocityTarget.IDLE))));
    driverA
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerve.zero();
                },
                swerve));
    // elevator commands
    driverB
        .a()
        .onTrue(new InstantCommand(() -> superstructure.setTargetState(SuperstructureState.AMP)));

    driverB
        .b()
        .onTrue(new InstantCommand(() -> superstructure.setTargetState(SuperstructureState.STOW)));
    driverB
        .y()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setTargetState(SuperstructureState.SUBWOOF_SHOT)));
    driverB
        .x()
        .onTrue(
            new InstantCommand(() -> superstructure.setTargetState(SuperstructureState.SHUTTLE)));
  }

  private void configureAutos() {}
}
