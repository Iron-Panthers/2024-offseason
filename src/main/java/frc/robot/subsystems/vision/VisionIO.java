package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;
    public List<PoseObservation> observations = new LinkedList<PoseObservation>();
  }

  default void updateInputs(VisionIOInputs inputs) {}

  // from EstimatedRobotPose
  public record PoseObservation(
      double timestamp,
      Pose3d estimatedPose,
      double ambiguity,
      int tagCount,
      double averageDistance,
      int[] tagIDs) {}
}
