package org.team1540.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOPhoton extends AprilTagVisionIO {
    PoseObservation last;
    PhotonCamera camera;
    private final Set<Integer> lastSeenTagIDs = new HashSet<>();
    protected final PhotonPoseEstimator poseEstimator;
    public Transform3d cameraTransformMeters;

    public AprilTagVisionIOPhoton(String cameraName, Transform3d cameraTransformMeters) {
        super(cameraName);
        camera = new PhotonCamera(cameraName);
        this.cameraTransformMeters = cameraTransformMeters;
        poseEstimator = new PhotonPoseEstimator(
                AprilTagVisionConstants.FieldConstants.aprilTagLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraTransformMeters);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        ArrayList<PoseObservation> poseObservations = new ArrayList<>();
        lastSeenTagIDs.clear();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> poseEstimatorResult = poseEstimator.update(result);
            if (poseEstimatorResult.isPresent()) {
                double totalDistance = 0.0;
                double totalAmbiguity = 0.0;
                for (PhotonTrackedTarget target : poseEstimatorResult.get().targetsUsed) {
                    lastSeenTagIDs.add(target.fiducialId);
                    totalDistance +=
                            target.getBestCameraToTarget().getTranslation().getNorm();
                    totalAmbiguity += target.poseAmbiguity;
                }
                poseObservations.add(new PoseObservation(
                        poseEstimatorResult.get().estimatedPose,
                        poseEstimatorResult.get().targetsUsed.size(),
                        totalDistance / poseEstimatorResult.get().targetsUsed.size(),
                        poseEstimatorResult.get().timestampSeconds,
                        totalAmbiguity / poseEstimatorResult.get().targetsUsed.size()));
            }
        }
        try {
            last = poseObservations.get(poseObservations.size()-1);
        } catch (Exception E) {
            last = null;
        }
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        inputs.seenTagIDs = lastSeenTagIDs.stream().mapToInt(Integer::intValue).toArray();
    }

}
