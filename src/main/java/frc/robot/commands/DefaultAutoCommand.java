package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.Drive;

public class DefaultAutoCommand extends SequentialCommandGroup {

  // dependency injection
  public DefaultAutoCommand(
      Rollers rollers, Drive swerve, Flywheels flywheels, Superstructure superstructure) {
    addCommands(
        new FunctionalCommand(
            () -> {},
            () -> rollers.setTargetState(RollerState.SPEAKER_TRANSFER),
            interrupted -> {
              rollers.setTargetState(RollerState.IDLE);
              flywheels.setVelocityTarget(Flywheels.VelocityTarget.SHOOT);
            },
            () -> rollers.acceleratorDetected(),
            rollers,
            flywheels),
        new WaitCommand(2.5),
        new InstantCommand(() -> rollers.setTargetState(RollerState.SHOOT_SPEAKER)));
  }
}
