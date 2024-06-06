package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public enum DriveModes {
    TELEOP,
    PATHPLANNER;
  }

  private DriveModes driveMode = DriveModes.TELEOP;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private Module[] modules = new Module[4];

  @AutoLogOutput(key = "Swerve/ArbitraryYaw")
  private Rotation2d arbitraryYaw = new Rotation2d();

  private ChassisSpeeds teleopTargetSpeeds = new ChassisSpeeds();
  private ChassisSpeeds pathplannerTargetSpeeds = new ChassisSpeeds();
  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;

    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);
  }

  @Override
  public void periodic() {
    // update inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    for (Module module : modules) {
      module.updateInputs();
    }

    switch (driveMode) {
      case TELEOP -> {
        targetSpeeds = teleopTargetSpeeds;
      }
      case PATHPLANNER -> {
        targetSpeeds = pathplannerTargetSpeeds;
      }
    }

    // run modules
    SwerveModuleState[] moduleTargetStates = Swerve.KINEMATICS.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleTargetStates, Swerve.Measurements.MAX_VELOCITY_MPS);

    for (int i = 0; i < modules.length; i++) {
      SwerveModuleState.optimize(moduleTargetStates[i], modules[i].getSteerHeading());
      modules[i].runToSetpoint(moduleTargetStates[i]);
    }
  }

  public void driveTeleopController(double xVelocity, double yVelocity, double radianVelocity) {
    if (driveMode != DriveModes.TELEOP) { // auto align override?
      driveMode = DriveModes.TELEOP;
    }
    this.teleopTargetSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, radianVelocity, arbitraryYaw);
  }

  public void drivePathplannerController(ChassisSpeeds trajectorySpeeds) {
    if (DriverStation.isAutonomousEnabled()) {
      driveMode = DriveModes.PATHPLANNER;
    }
    this.pathplannerTargetSpeeds = trajectorySpeeds;
  }
}
