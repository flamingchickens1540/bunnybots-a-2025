package org.team1540.robot.subsystems.vision.apriltag;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.Constants;
import org.team1540.robot.RobotState;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagVisionIO frontLeftCameraIO;
    private final AprilTagVisionIO frontRightCameraIO;
    private final PhotonVisionIOInputsAutoLogged frontRightCameraInputs = new PhotonVisionIOInputsAutoLogged();
    private final PhotonVisionIOInputsAutoLogged frontLeftCameraInputs = new PhotonVisionIOInputsAutoLogged();

    public AprilTagVision(AprilTagVisionIO frontLeftCameraIO, AprilTagVisionIO frontRightCameraIO) {
        this.frontLeftCameraIO = frontLeftCameraIO;
        this.frontRightCameraIO = frontRightCameraIO;
    }

    public static AprilTagVision createReal() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real vision on simulated robot", false);
        }
        return new AprilTagVision(
                new AprilTagVisionIOPhoton(
                        "front-left", AprilTagVisionConstants.cameraOffsets.frontLeftCameraPLaceOnRobot),
                new AprilTagVisionIOPhoton(
                        "front-right", AprilTagVisionConstants.cameraOffsets.frontRightCameraPLaceOnRobot));
    }

    public static AprilTagVision createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            DriverStation.reportWarning("Using dummy vision on real robot", false);
        }
        return new AprilTagVision(new AprilTagVisionIO("front-left") {}, new AprilTagVisionIO("front-right") {});
    }

    @Override
    public void periodic() {

        frontLeftCameraIO.updateInputs(frontLeftCameraInputs);
        frontRightCameraIO.updateInputs(frontRightCameraInputs);
        Logger.processInputs("Vision/RightCamera", frontRightCameraInputs);
        Logger.processInputs("Vision/LeftCamera", frontLeftCameraInputs);
        for (AprilTagVisionIO.PoseObservation poseObservation : frontLeftCameraInputs.poseObservations) {
            RobotState.getInstance().addVisionMeasurement(poseObservation);
        }
        for (AprilTagVisionIO.PoseObservation poseObservation : frontRightCameraInputs.poseObservations) {
            RobotState.getInstance().addVisionMeasurement(poseObservation);
        }
    }
}
