// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.RGBSubsystem;
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
import java.util.function.DoubleSupplier;

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
  private RGBSubsystem rgbSubsystem;

  public RobotContainer() {
    Intake intake = null;
    Accelerator accelerator = null;
    Serializer serializer = null;
    RollerSensorsIO rollerSensorsIO = null;
    rgbSubsystem = new RGBSubsystem();

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
                  swerve.driveTeleopController(-driverA.getLeftY(), -driverA.getLeftX());
                })
            .withName("Drive Teleop"));

    DoubleSupplier rotationAbsolute =
        () -> driverA.getRightTriggerAxis() - driverA.getLeftTriggerAxis();

    new Trigger(() -> Math.abs(rotationAbsolute.getAsDouble()) > 0.07)
        .whileTrue(
            new FunctionalCommand(
                () -> {},
                () ->
                    swerve.driveAnglePeriodic(
                        driverA.getLeftY(),
                        driverA.getLeftX(),
                        swerve.getTargetAngle()
                            + 5
                                * Math.signum(
                                    rotationAbsolute.getAsDouble()
                                        * Math.pow(Math.abs(rotationAbsolute.getAsDouble()), 1.5))),
                interrupted -> {},
                () -> false,
                swerve));

    // intake note and then outtake for a little time
    driverA
        .leftBumper()
        .onTrue(
            new InstantCommand(() -> superstructure.setTargetState(SuperstructureState.INTAKE))
                .andThen(
                    new FunctionalCommand(
                        () -> rollers.setTargetState(RollerState.INTAKE),
                        () -> {},
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> rollers.serializerDetected(),
                        rollers))
                .andThen(
                    new InstantCommand(
                        () ->
                            rgbSubsystem.showMessage(
                                RGBSubsystem.Lights.Colors.RED,
                                RGBSubsystem.PatternTypes.STROBE,
                                RGBSubsystem.MessagePriority.F_NOTE_IN_ROBOT)))
                .andThen(
                    new InstantCommand(() -> rollers.setTargetState(RollerState.AMP_EJECT), rollers)
                        .withTimeout(0.6))
                .andThen(
                    new FunctionalCommand(
                            () -> rollers.setTargetState(RollerState.EJECT),
                            () -> {},
                            interrupted -> rollers.setTargetState(RollerState.IDLE),
                            () -> false,
                            rollers)
                        .withTimeout(2))
                .andThen(new InstantCommand(() -> rollers.setTargetState(RollerState.IDLE))));
    driverB
        .leftBumper()
        .onTrue(
            new InstantCommand(() -> superstructure.setTargetState(SuperstructureState.INTAKE))
                .andThen(
                    new FunctionalCommand(
                        () -> rollers.setTargetState(RollerState.INTAKE),
                        () -> {},
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> rollers.serializerDetected(),
                        rollers))
                .andThen(
                    new InstantCommand(
                        () ->
                            rgbSubsystem.showMessage(
                                RGBSubsystem.Lights.Colors.RED,
                                RGBSubsystem.PatternTypes.STROBE,
                                RGBSubsystem.MessagePriority.F_NOTE_IN_ROBOT)))
                .andThen(
                    new InstantCommand(() -> rollers.setTargetState(RollerState.AMP_EJECT), rollers)
                        .withTimeout(0.6))
                .andThen(
                    new FunctionalCommand(
                            () -> rollers.setTargetState(RollerState.EJECT),
                            () -> {},
                            interrupted -> rollers.setTargetState(RollerState.IDLE),
                            () -> false,
                            rollers)
                        .withTimeout(2))
                .andThen(new InstantCommand(() -> rollers.setTargetState(RollerState.IDLE))));
    // eject note manual command
    driverB
        .leftTrigger()
        .onTrue(
            new FunctionalCommand(
                    () -> rollers.setTargetState(RollerState.EJECT),
                    () -> {},
                    interrupted -> rollers.setTargetState(RollerState.IDLE),
                    () -> false,
                    rollers)
                .withTimeout(2));

    // Shoot command (either amp or speaker)
    driverA
        .rightBumper()
        .onTrue(
            new InstantCommand(
                    () ->
                        rollers.setTargetState(
                            rollers.acceleratorDetected() && flywheels.atSpeed()
                                ? RollerState.SHOOT_SPEAKER
                                : superstructure.getTargetState() == SuperstructureState.AMP
                                    ? RollerState.SHOOT_AMP
                                    : rollers.getTargetState()),
                    rollers)
                .andThen(new WaitCommand(1))
                .andThen(
                    new InstantCommand(
                        () -> {
                          if (rollers.getTargetState() == RollerState.SHOOT_SPEAKER
                              || rollers.getTargetState() == RollerState.SHOOT_AMP) {
                            rollers.setTargetState(RollerState.IDLE);
                            flywheels.setVelocityTarget(Flywheels.VelocityTarget.IDLE);
                            superstructure.setTargetState(SuperstructureState.STOW);
                          }
                        }))
                .andThen(new InstantCommand(() -> rgbSubsystem.expireCurrent())));
    driverB
        .rightBumper()
        .onTrue(
            new InstantCommand(
                    () ->
                        rollers.setTargetState(
                            rollers.acceleratorDetected()
                                ? RollerState.SHOOT_SPEAKER
                                : superstructure.getTargetState() == SuperstructureState.AMP
                                    ? RollerState.SHOOT_AMP
                                    : rollers.getTargetState()),
                    rollers)
                .andThen(new WaitCommand(1))
                .andThen(
                    new InstantCommand(
                        () -> {
                          if (rollers.getTargetState() == RollerState.SHOOT_SPEAKER
                              || rollers.getTargetState() == RollerState.SHOOT_AMP) {
                            rollers.setTargetState(RollerState.IDLE);
                            flywheels.setVelocityTarget(Flywheels.VelocityTarget.IDLE);
                            superstructure.setTargetState(SuperstructureState.STOW);
                          }
                        }))
                .andThen(new InstantCommand(() -> rgbSubsystem.expireCurrent())));

    // transfer note to shooter
    driverB
        .b()
        .onTrue(
            new FunctionalCommand(
                    () -> superstructure.setTargetState(SuperstructureState.STOW),
                    () -> {},
                    interrupted -> {},
                    () ->
                        superstructure.elevatorPosition() < 1
                            && superstructure.getTargetState() == SuperstructureState.STOW)
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
    // speaker shot pivot snap
    driverB
        .y()
        .onTrue(
            // new FunctionalCommand(
            //         () -> {},
            //         () -> swerve.driveAnglePeriodic(driverA.getLeftX(), driverA.getLeftY(), 180),
            //         interrupted -> {},
            //         () -> Math.abs(swerve.getAngularError(0)) < 1,
            //         swerve)
            //     .alongWith(
            new InstantCommand(
                    () -> superstructure.setTargetState(SuperstructureState.SUBWOOF_SHOT))
                .alongWith(
                    new InstantCommand(
                        () -> flywheels.setVelocityTarget(Flywheels.VelocityTarget.SHOOT))));
    // Initiate amp shot
    driverB
        .a()
        .onTrue(
            new InstantCommand(
                    () ->
                        superstructure.setTargetState(
                            superstructure.getTargetState() == SuperstructureState.AMP
                                ? SuperstructureState.AMP
                                : SuperstructureState.INTAKE))
                .andThen(
                    new FunctionalCommand(
                        () -> {},
                        () -> {
                          if (superstructure.getTargetState() == SuperstructureState.AMP) {
                            rollers.setTargetState(RollerState.IDLE);
                          } else {
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
                          }
                        },
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> rollers.getTargetState() == RollerState.IDLE,
                        rollers))
                .andThen(
                    new FunctionalCommand(
                        () -> {},
                        () -> {
                          if (superstructure.getTargetState() == SuperstructureState.AMP) {
                            rollers.setTargetState(RollerState.IDLE);
                          } else {
                            rollers.setTargetState(RollerState.AMP_TRANSFER);
                          }
                        },
                        interrupted -> rollers.setTargetState(RollerState.IDLE),
                        () -> !rollers.serializerDetected(),
                        rollers))
                .andThen(
                    new InstantCommand(
                        () -> {
                          if (superstructure.getTargetState() == SuperstructureState.AMP) {
                            rollers.setTargetState(RollerState.IDLE);
                          } else {
                            rollers.setTargetState(RollerState.EJECT);
                          }
                        },
                        rollers))
                .andThen(new WaitCommand(0.08))
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

    // Initiate subwoofer shot

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
    // zeroing
    driverA
        .start()
        .onTrue(
            new FunctionalCommand(
                    () -> {
                      swerve.zero();
                      //   superstructure.setTargetState(SuperstructureState.ZERO);
                    },
                    () -> {},
                    interrupted -> {},
                    () ->
                        // superstructure.getElevatorSupplyCurrentAmps() > 4
                        //     && superstructure.getPivotSupplyCurrentAmps() > 4
                        true,
                    swerve,
                    superstructure)
                .andThen(
                    new InstantCommand(
                        () -> superstructure.setTargetState(SuperstructureState.STOW),
                        superstructure)));

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
                },
                rollers,
                flywheels,
                superstructure));
    driverA
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  rollers.setTargetState(RollerState.IDLE);
                  ;
                  flywheels.setVelocityTarget(Flywheels.VelocityTarget.IDLE);
                  superstructure.setTargetState(SuperstructureState.STOP);
                },
                rollers,
                flywheels,
                superstructure));
    driverB
        .povDown()
        .onTrue(new InstantCommand(() -> superstructure.setTargetState(SuperstructureState.STOW)));
    driverB
        .povUp()
        .onTrue(new InstantCommand(() -> superstructure.setTargetState(SuperstructureState.AMP)));

    // turning setpoints
    // source
    driverA
        .povUp()
        .onTrue(
            new FunctionalCommand(
                () -> {},
                () ->
                    swerve.driveAnglePeriodic(
                        driverA.getLeftX(),
                        driverA.getLeftY(),
                        DriverStation.getAlliance().get().equals(Alliance.Red) ? -39 : 39),
                interrupted -> {},
                () -> Math.abs(swerve.getAngularError(0)) < 5));
    // amp
    driverA
        .povLeft()
        .onTrue(
            new FunctionalCommand(
                () -> {},
                () ->
                    swerve.driveAnglePeriodic(
                        driverA.getLeftX(),
                        driverA.getLeftY(),
                        DriverStation.getAlliance().get().equals(Alliance.Red) ? -90 : 90),
                interrupted -> {},
                () -> Math.abs(swerve.getAngularError(0)) < 5));
    // shuttle
    driverA
        .povRight()
        .onTrue(
            new FunctionalCommand(
                () -> {},
                () -> {
                  swerve.driveAnglePeriodic(
                      driverA.getLeftX(),
                      driverA.getLeftY(),
                      DriverStation.getAlliance().get().equals(Alliance.Red) ? -50 : 50);
                  superstructure.setTargetState(SuperstructureState.SHUTTLE);
                },
                interrupted -> {},
                () -> Math.abs(swerve.getAngularError(0)) < 5,
                superstructure));
    // speaker
    driverA
        .povDown()
        .onTrue(
            new FunctionalCommand(
                () -> {},
                () -> swerve.driveAnglePeriodic(driverA.getLeftX(), driverA.getLeftY(), 0),
                interrupted -> {},
                () -> Math.abs(swerve.getAngularError(0)) < 5));
  }

  private void configureAutos() {}
}
