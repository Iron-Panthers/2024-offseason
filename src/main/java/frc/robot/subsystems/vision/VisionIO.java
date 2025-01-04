package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;
    // data from EstimatedRobotPose
    public double timestamp = 0;
    public Pose3d estimatedPose = new Pose3d();
    public double ambiguity = 0;
    public int tagCount = 0;
    public double averageDistance = 0;
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
