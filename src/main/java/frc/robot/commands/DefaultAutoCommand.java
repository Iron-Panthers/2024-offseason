package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.flywheels.Flywheels.VelocityTarget;
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


public class DefaultAutoCommand extends Command {


 private Rollers rollers;
 private Drive swerve;
 private Flywheels flywheels;

 //dependency injection
 public DefaultAutoCommand(Rollers rollers, Drive swerve, Flywheels flywheels) {
  this.rollers = rollers;
  this.swerve = swerve;
  this.flywheels = flywheels;
  addRequirements(rollers);
  addRequirements(swerve);
  addRequirements(flywheels);
 }


/* Initialize: useful for performing tasks that only
 * need to be performed once per time scheduled, such as setting motors to run at a 
 * constant speed, etc. etc.*/
 @Override
  public void initialize() {}

/* Execute: Should be used for any task that needs to be done 
 * continually while the command is scheduled, such as updating motor
 * outputs to match joystick inputs (kind of like periodic I guess?) */
 @Override
 public void execute() {}

 //when the command should end:
  @Override
  public boolean isFinished() {
    return false; //for now....
    //return something (true when it's finished)
  }

 //FIXME: MAKE SURE TO PUT THE CODE BELOW SOMEWHERE THIS IS WHAT THE COMMAND IS SUPPOSED TO DO!!!!!!
//  /*Steps:
//        * 1. Intake the note a little bit
//        * 2. Shoot the note
//        * 3. Move away from the speaker a little bit (0.25 power for 1.5 seconds)
//        * 4. Turn to an angle parallel to the sides of the field
//        * 5. Move backwards (0.5 power for 2 seconds) */
//    if(rollers.isContactingNote() == true) {
//            rollers.setTargetCommand(RollerState.INTAKE);
//    } else {
//            rollers.setTargetCommand(RollerState.IDLE);
//    }
//    new WaitCommand(1.0);
//    flywheels.setVelocityTarget(VelocityTarget.SHOOT);
//    new WaitCommand(2.5);
//    swerve.setVelocityTarget(-0.25, 0, 0);
//    new WaitCommand(2.5);
//    swerve.driveAnglePeriodic(0, 0, 0);
//    new WaitCommand(2.5);
//    swerve.setVelocityTarget(-0.5, 0, 0);
//    //FIXME: FINISH THE MANUAL AUTO SEQUENCE


}



