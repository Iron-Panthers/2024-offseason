package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
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
    TRAJECTORY;
  }

  private DriveModes driveMode = DriveModes.TELEOP;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private Module[] modules = new Module[4];

  @AutoLogOutput(key = "Swerve/ArbitraryYaw")
  private Rotation2d arbitraryYaw = new Rotation2d();

  @AutoLogOutput(key = "Swerve/YawOffset")
  private Rotation2d gyroYawOffset = new Rotation2d(0);

  private ChassisSpeeds teleopTargetSpeeds = new ChassisSpeeds();
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

    arbitraryYaw =
        Rotation2d.fromDegrees(
            (gyroInputs.yawPosition.minus(gyroYawOffset).getDegrees() % 360 + 360) % 360);

    for (Module module : modules) {
      module.updateInputs();
    }

    switch (driveMode) {
      case TELEOP -> {
        targetSpeeds = teleopTargetSpeeds;
      }
      case TRAJECTORY -> {}
    }

    // run modules
    SwerveModuleState[] moduleTargetStates = Swerve.KINEMATICS.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleTargetStates, Swerve.DRIVE_CONFIG.maxLinearVelocity());

    for (int i = 0; i < modules.length; i++) {
      SwerveModuleState.optimize(moduleTargetStates[i], modules[i].getSteerHeading());
      modules[i].runToSetpoint(moduleTargetStates[i]);
    }

    Logger.recordOutput("Swerve/TargetSpeeds", targetSpeeds);
    Logger.recordOutput("Swerve/DriveMode", driveMode);
    // also swerve states?
  }

  public void driveTeleopController(double xAxis, double yAxis, double omega) {
    if (driveMode != DriveModes.TELEOP) { // auto align override?
      driveMode = DriveModes.TELEOP;
    }

    double xVelocity =
        MathUtil.applyDeadband(Math.copySign(xAxis * xAxis, xAxis), 0.07)
            * Swerve.DRIVE_CONFIG.maxLinearVelocity();
    double yVelocity =
        MathUtil.applyDeadband(Math.copySign(yAxis * yAxis, yAxis), 0.07)
            * Swerve.DRIVE_CONFIG.maxLinearVelocity();
    double radianVelocity =
        MathUtil.applyDeadband(Math.copySign(omega * omega, omega), 0.07)
            * Swerve.DRIVE_CONFIG.maxAngularVelocity();

    this.teleopTargetSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, radianVelocity, arbitraryYaw);
  }

  public void setTrajectoryFollower(ChassisSpeeds trajectorySpeeds) {
    if (DriverStation.isAutonomousEnabled()) {
      driveMode = DriveModes.TRAJECTORY;
    }
  }
}
