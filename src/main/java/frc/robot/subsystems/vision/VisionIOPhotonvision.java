package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonvision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;

  public VisionIOPhotonvision(AprilTagFieldLayout fieldLayout, int index) {
    camera = new PhotonCamera("photonvision-" + index);
    estimator =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            new Transform3d()); // FIXME transform
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return;

    Optional<EstimatedRobotPose> optEstimation = estimator.update(result);
    if (optEstimation.isEmpty()) return;
    EstimatedRobotPose estimation = optEstimation.get();

    double totalDistance = 0;
    for (var target : result.getTargets()) {
      totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }

    inputs.timestamp = result.getTimestampSeconds();
    inputs.estimatedPose = estimation.estimatedPose;
    inputs.ambiguity = result.getMultiTagResult().estimatedPose.ambiguity;
    inputs.tagCount = result.getMultiTagResult().fiducialIDsUsed.size();
    inputs.averageDistance = totalDistance / result.getTargets().size();
  }
}
