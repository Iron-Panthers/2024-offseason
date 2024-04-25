package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
  private static final int id = 0;

  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(id);
    pigeon.optimizeBusUtilization();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.setYaw(0);

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    BaseStatusSignal.setUpdateFrequencyForAll(100, yaw, yawVelocity);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}
