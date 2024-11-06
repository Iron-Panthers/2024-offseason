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
import frc.robot.subsystems.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.rollers.RollerSensorsIO;
import frc.robot.subsystems.rollers.RollerSensorsIOComp;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.rollers.accelerator.Accelerator;
import frc.robot.subsystems.rollers.accelerator.AcceleratorIOTalonFX;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIOTalonFX;
import frc.robot.subsystems.rollers.serializer.Serializer;
import frc.robot.subsystems.rollers.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotIOTalonFX;
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

  public RobotContainer() {
    Intake intake = null;
    Accelerator accelerator = null;
    Serializer serializer = null;
    RollerSensorsIO rollerSensorsIO = null;

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
          rollerSensorsIO = new RollerSensorsIOComp();

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

    superstructure =
        new Superstructure(new Elevator(new ElevatorIOTalonFX()), new Pivot(new PivotIOTalonFX()));
    rollers = new Rollers(intake, accelerator, serializer, rollerSensorsIO);

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

    // intake note and then outtake for a little time
    driverA
        .leftBumper()
        .onTrue(
            new FunctionalCommand(
                    () -> superstructure.setTargetState(SuperstructureState.INTAKE),
                    () -> {},
                    interrupted -> {},
                    () -> superstructure.atPosition())
                .andThen(
                    new FunctionalCommand(
                        () -> rollers.setTargetState(RollerState.INTAKE),
                        () -> {},
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> rollers.serializerDetected(),
                        rollers))
                .andThen(
                    new InstantCommand(() -> rollers.setTargetState(RollerState.EJECT), rollers)
                        .withTimeout(0.6))
                .andThen(new InstantCommand(() -> rollers.setTargetState(RollerState.IDLE))));
    driverB
        .leftBumper()
        .onTrue(
            new FunctionalCommand(
                    () -> superstructure.setTargetState(SuperstructureState.INTAKE),
                    () -> {},
                    interrupted -> {},
                    () -> superstructure.atPosition())
                .andThen(
                    new FunctionalCommand(
                        () -> rollers.setTargetState(RollerState.INTAKE),
                        () -> {},
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> rollers.serializerDetected(),
                        rollers))
                .andThen(
                    new InstantCommand(() -> rollers.setTargetState(RollerState.EJECT), rollers)
                        .withTimeout(0.6))
                .andThen(new InstantCommand(() -> rollers.setTargetState(RollerState.IDLE))));

    // Shoot command (either amp or speaker)
    driverA
        .rightBumper()
        .onTrue(
            new InstantCommand(
                    () ->
                        rollers.setTargetState(
                            rollers.acceleratorDetected() && flywheels.atSpeed()
                                ? RollerState.SHOOT_SPEAKER
                                : superstructure.atPosition()
                                        && superstructure.getTargetState()
                                            == SuperstructureState.AMP
                                    ? RollerState.SHOOT_AMP
                                    : rollers.getTargetState()),
                    rollers)
                .andThen(new WaitCommand(2))
                .andThen(
                    new InstantCommand(
                        () -> {
                          if (rollers.getTargetState() == RollerState.SHOOT_SPEAKER
                              || rollers.getTargetState() == RollerState.SHOOT_AMP) {
                            rollers.setTargetState(RollerState.IDLE);
                            flywheels.setVelocityTarget(Flywheels.VelocityTarget.IDLE);
                            superstructure.setTargetState(SuperstructureState.STOW);
                          }
                        })));
    // transfer note to shooter
    driverA
        .b()
        .onTrue(
            new FunctionalCommand(
                    () -> superstructure.setTargetState(SuperstructureState.STOW),
                    () -> {},
                    interrupted -> {},
                    () -> superstructure.atPosition())
                .andThen(new WaitCommand(1))
                .andThen(
                    new FunctionalCommand(
                        () -> {},
                        () -> rollers.setTargetState(RollerState.SPEAKER_TRANSFER),
                        interrupted -> {
                          rollers.setTargetState(RollerState.IDLE);
                          flywheels.setVelocityTarget(Flywheels.VelocityTarget.SHOOT);
                        },
                        () -> rollers.acceleratorDetected(),
                        rollers,
                        flywheels)));
    driverB
        .b()
        .onTrue(
            new FunctionalCommand(
                    () -> superstructure.setTargetState(SuperstructureState.STOW),
                    () -> {},
                    interrupted -> {},
                    () -> superstructure.atPosition())
                .andThen(
                    new FunctionalCommand(
                        () -> {},
                        () -> rollers.setTargetState(RollerState.SPEAKER_TRANSFER),
                        interrupted -> {
                          rollers.setTargetState(RollerState.IDLE);
                          flywheels.setVelocityTarget(Flywheels.VelocityTarget.SHOOT);
                        },
                        () -> rollers.acceleratorDetected(),
                        rollers,
                        flywheels)));
    // Initiate amp shot
    driverA
        .x()
        .onTrue(
            new FunctionalCommand(
                    () -> superstructure.setTargetState(SuperstructureState.INTAKE),
                    () -> {},
                    interrupted -> {},
                    () -> superstructure.atPosition())
                .andThen(
                    new FunctionalCommand(
                        () -> {},
                        () -> {
                          boolean once = false;
                          rollers.setTargetState(
                              rollers.acceleratorDetected()
                                  ? RollerState.AMP_TRANSFER
                                  : RollerState.INTAKE);
                          if (rollers.getTargetState() == RollerState.AMP_TRANSFER
                              && rollers.serializerDetected()) {
                            once = true;
                          }
                          if (once && !rollers.serializerDetected()) {
                            rollers.setTargetState(RollerState.INTAKE);
                          }
                          if (rollers.serializerDetected()
                              && rollers.getTargetState() == RollerState.INTAKE) {
                            rollers.setTargetState(RollerState.IDLE);
                          }
                        },
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> rollers.getTargetState() == RollerState.IDLE,
                        rollers))
                .andThen(
                    new FunctionalCommand(
                        () -> {},
                        () -> rollers.setTargetState(RollerState.AMP_TRANSFER),
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> !rollers.serializerDetected(),
                        rollers))
                .andThen(
                    new InstantCommand(() -> rollers.setTargetState(RollerState.EJECT), rollers)
                        .withTimeout(0.7))
                .andThen(
                    new InstantCommand(
                        () -> {
                          superstructure.setTargetState(SuperstructureState.AMP);
                          flywheels.setVelocityTarget(Flywheels.VelocityTarget.IDLE);
                          rollers.setTargetState(RollerState.IDLE);
                        },
                        superstructure,
                        rollers,
                        flywheels)));
    driverB
        .a()
        .onTrue(
            new FunctionalCommand(
                    () -> superstructure.setTargetState(SuperstructureState.INTAKE),
                    () -> {},
                    interrupted -> {},
                    () -> superstructure.atPosition())
                .andThen(
                    new FunctionalCommand(
                        () -> {},
                        () ->
                            rollers.setTargetState(
                                rollers.acceleratorDetected()
                                    ? RollerState.AMP_TRANSFER
                                    : RollerState.INTAKE),
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> rollers.serializerDetected(),
                        rollers))
                .andThen(
                    new FunctionalCommand(
                        () -> {},
                        () -> rollers.setTargetState(RollerState.AMP_TRANSFER),
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> !rollers.serializerDetected(),
                        rollers))
                .andThen(
                    new InstantCommand(
                        () -> superstructure.setTargetState(SuperstructureState.AMP),
                        superstructure)));
    // Initiate subwoofer shot
    driverA
        .a()
        .onTrue(
            new FunctionalCommand(
                    () -> {},
                    () -> swerve.driveAnglePeriodic(driverA.getLeftX(), driverA.getLeftY(), 0),
                    interrupted -> {},
                    () -> Math.abs(swerve.getAngularError(0)) < 1,
                    swerve)
                .alongWith(
                    new InstantCommand(
                        () -> superstructure.setTargetState(SuperstructureState.SUBWOOF_SHOT))));
    // -----Flywheel Controls-----
    //
    // driverA
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               flywheels.setVelocityTarget(VelocityTarget.IDLE);
    //             },
    //             flywheels));

    driverA
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerve.zero();
                  //   superstructure.setTargetState(SuperstructureState.ZERO);
                  ;
                },
                swerve,
                superstructure));
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
    // cancel everything
    driverB
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  rollers.setTargetState(RollerState.IDLE);
                  ;
                  flywheels.setVelocityTarget(Flywheels.VelocityTarget.IDLE);
                  superstructure.setTargetState(SuperstructureState.STOP);
                }));
  }

  private void configureAutos() {}
}
