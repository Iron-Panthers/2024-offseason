package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;

public class ModuleIOFalcon500 implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX steerTalon;
  private final CANcoder encoder;

  public ModuleIOFalcon500(ModuleConfig config) {
    driveTalon = new TalonFX(config.driveID());
    steerTalon = new TalonFX(config.steerID());
    encoder = new CANcoder(config.encoderID());
  }
}
