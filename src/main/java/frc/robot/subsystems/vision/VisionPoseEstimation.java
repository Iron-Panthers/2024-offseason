package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionPoseEstimation extends SubsystemBase {
  private ArrayList<PhotonCamera> cameras;

  private static final Transform3d CAMERA1_TO_CENTER =
      new Transform3d( // FIXME
          new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  private static final Transform3d CAMERA2_TO_CENTER =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  private ArrayList<PhotonPoseEstimator> poseEstimators;

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // FIXME

  public VisionPoseEstimation() {
    addCamera("Camera1", CAMERA1_TO_CENTER); // FIXME
    addCamera("Camera2", CAMERA2_TO_CENTER); // FIXME
  }

  public void addCamera(String cameraName, Transform3d robotToCamera) {
    PhotonCamera camera = new PhotonCamera(cameraName);
    cameras.add(camera);
    PhotonPoseEstimator poseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    poseEstimator.setRobotToCameraTransform(robotToCamera);
    poseEstimators.add(poseEstimator);
  }

  @Override
  public void periodic() {
    visionUpdatePose();
  }

  public void visionUpdatePose() {
    for (int i = 0; i < cameras.size(); i++) {
      final Optional<EstimatedRobotPose> optionalEstimatedPose;
      if (cameras.get(i).getAllUnreadResults().size() > 0) {
        PhotonPipelineResult result = cameras.get(i).getAllUnreadResults().get(0);
        optionalEstimatedPose = poseEstimators.get(i).update(result);
      } else {
        optionalEstimatedPose = Optional.empty();
      }

      if (optionalEstimatedPose.isPresent()) {
        final EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();
        RobotState.getInstance().addVisionMeasurement(estimatedPose);
      }
    }
  }
}
