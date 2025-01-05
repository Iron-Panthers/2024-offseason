package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionPoseEstimation extends SubsystemBase {
  private ArrayList<PhotonCamera> cameras;

  private Pose2d visionEstimatedPose = new Pose2d();


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
    double xSum = 0;
    double ySum = 0;
    double angleSum = 0;
    int targets = 0;
    for (int i = 0; i < cameras.size(); i++) {
      final Optional<EstimatedRobotPose> optionalEstimatedPose;
      if (cameras.get(i).getAllUnreadResults().size() > 0) {
        PhotonPipelineResult result = cameras.get(i).getAllUnreadResults().get(0);
        optionalEstimatedPose = poseEstimators.get(i).update(result);
        EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();
        RobotState.getInstance().addVisionMeasurement(estimatedPose);

        estimatedPose.estimatedPose.toPose2d();
        xSum += estimatedPose.estimatedPose.toPose2d().getX();
        ySum += estimatedPose.estimatedPose.toPose2d().getY();
        angleSum += estimatedPose.estimatedPose.toPose2d().getRotation().getRadians();
        targets++;
      }
    }
    if (targets > 0) {
      xSum /= targets;
      ySum /= targets;
      angleSum /= targets;
      visionEstimatedPose = new Pose2d(xSum, ySum, new Rotation2d(angleSum));
    }
  }


  @AutoLogOutput(key = "RobotState/VisionPose")
  public Pose2d getVisionPose() {
    return visionEstimatedPose;
  }
}
