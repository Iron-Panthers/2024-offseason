package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionPoseEstimation extends SubsystemBase{
    private ArrayList<PhotonCamera> cameras;

    private ArrayList<PhotonPoseEstimator> poseEstimators;
    private SwerveDrivePoseEstimator masterPoseEstimator;
    
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);//FIXME

    private final Transform3d robotToCamera = new Transform3d(0,0,0, new Rotation3d());


    public VisionPoseEstimation(SwerveDrivePoseEstimator masterPoseEstimator){
        this.masterPoseEstimator = masterPoseEstimator;
    }

    public void addCamera(String cameraName){
        PhotonCamera camera = new PhotonCamera(cameraName);
        cameras.add(camera);
        poseEstimators.add(new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera));
    }

    @Override
    public void periodic(){
        for(int i = 0; i<cameras.size(); i++){
            PhotonPipelineResult result = cameras.get(i).getAllUnreadResults().get(0);
            final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = poseEstimators.get(i).update(result);
            if (optionalEstimatedPoseRight.isPresent()) {
                final EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();          
                masterPoseEstimator.addVisionMeasurement(
                    estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds);
            }
        }
    }

    public Pose2d getRobotPose(){
        return masterPoseEstimator.getEstimatedPosition();
    }
}
