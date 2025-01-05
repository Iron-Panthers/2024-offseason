package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.swerve.DriveConstants.KINEMATICS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.controllers.HeadingController;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.controllers.TeleopController;
import java.util.Arrays;
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

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  private final TeleopController teleopController;

  private HeadingController headingController;

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;

    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);

    teleopController = new TeleopController();
    headingController = new HeadingController(new Rotation2d());
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

    // pass odometry data to robotstate
    SwerveModulePosition[] wheelPositions =
        Arrays.stream(modules)
            .map(module -> module.getModulePosition())
            .toArray(SwerveModulePosition[]::new);
    RobotState.getInstance()
        .addOdometryMeasurement(
            new RobotState.OdometryMeasurement(
                wheelPositions, gyroInputs.yawPosition, Timer.getFPGATimestamp()));

    switch (driveMode) {
      case TELEOP -> {
        targetSpeeds = teleopController.update(arbitraryYaw);
        if (headingController != null) {
          targetSpeeds.omegaRadiansPerSecond =
              headingController.update(arbitraryYaw, gyroInputs.yawVelocityRadPerSec).getRadians();
        }
      }
      case TRAJECTORY -> {}
    }

    // run modules

    /* use kinematics to get desired module states */
    ChassisSpeeds discretizedSpeeds =
        ChassisSpeeds.discretize(targetSpeeds, Constants.PERIODIC_LOOP_SEC);
    /* ChassisSpeeds discretizedSpeeds = targetSpeeds; // FIXME
    discretizedSpeeds.discretize(Constants.PERIODIC_LOOP_SEC); */

    SwerveModuleState[] moduleTargetStates = KINEMATICS.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleTargetStates, DRIVE_CONFIG.maxLinearVelocity());

    SwerveModuleState[] optimizedTargetStates = new SwerveModuleState[4];

    for (int i = 0; i < modules.length; i++) {
      optimizedTargetStates[i] = moduleTargetStates[i];
      optimizedTargetStates[i].optimize(modules[i].getSteerHeading());
      modules[i].runToSetpoint(optimizedTargetStates[i]);
    }

    Logger.recordOutput("Swerve/ModuleStates/Optimized", optimizedTargetStates);
    Logger.recordOutput("Swerve/TargetSpeeds", targetSpeeds);
    Logger.recordOutput("Swerve/DriveMode", driveMode);
  }

  public void driveTeleopController(double xAxis, double yAxis, double omega) {
    if (DriverStation.isTeleopEnabled()) {
      if (driveMode != DriveModes.TELEOP) {
        driveMode = DriveModes.TELEOP;
      }
      if (headingController != null) {
        headingController = null;
      }

      teleopController.acceptJoystickInput(xAxis, yAxis, omega);
    }
  }
  /*radians*/
  public void driveHeadingController(double xAxis, double yAxis, Rotation2d omega) {
    if (DriverStation.isTeleopEnabled()) {
      if (driveMode != DriveModes.TELEOP) {
        driveMode = DriveModes.TELEOP;
      }
      if (headingController == null) {
        headingController = new HeadingController(omega);
      }
      headingController.setTarget(omega);
      teleopController.acceptJoystickInput(xAxis, yAxis, 0);
    }
  }
  /*radians*/
  public void driveHeadingChangeController(double xAxis, double yAxis, double deltaOmega) {
    if (DriverStation.isTeleopEnabled()) {
      if (driveMode != DriveModes.TELEOP) {
        driveMode = DriveModes.TELEOP;
      }
      if (headingController == null) {
        headingController = new HeadingController(new Rotation2d());
      }
      headingController.changeTarget(deltaOmega);
      teleopController.acceptJoystickInput(xAxis, yAxis, 0);
    }
  }

  public void setTrajectoryFollower(ChassisSpeeds trajectorySpeeds) {
    if (DriverStation.isAutonomousEnabled()) {
      driveMode = DriveModes.TRAJECTORY;
    }
  }

  private void zeroGyro() {
    gyroYawOffset = gyroInputs.yawPosition;
    if (headingController != null) {
      headingController.setTarget(new Rotation2d());
    }
  }

  public Command zeroGyroCommand() {
    return this.runOnce(() -> zeroGyro());
  }
}
