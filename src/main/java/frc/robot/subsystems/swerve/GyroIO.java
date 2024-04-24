package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0;
  }

  default void updateInputs(GyroIOInputs inputs) {}
}
