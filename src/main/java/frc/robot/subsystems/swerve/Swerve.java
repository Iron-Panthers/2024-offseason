package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

  private Module[] modules = new Module[4];

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  public Swerve(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);
  }

  @Override
  public void periodic() {}

  public void drive(ChassisSpeeds targetSpeeds) {
    this.targetSpeeds = targetSpeeds
  }
}
