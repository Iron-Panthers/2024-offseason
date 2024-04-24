package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon2 implements GyroIO {
  private static final int id = 0;

  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(id);

    yaw = pigeon.getYaw();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValue());
  }
}
