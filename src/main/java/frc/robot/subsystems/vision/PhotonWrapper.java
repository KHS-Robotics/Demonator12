package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class PhotonWrapper {
    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;

    public PhotonWrapper(String cameraName) {
        camera = new PhotonCamera(cameraName);

        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, Constants.CAMERA_1_POS);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousEstimatedPose) {
        photonPoseEstimator.setReferencePose(previousEstimatedPose);
        return photonPoseEstimator.update();
    }
    
}
