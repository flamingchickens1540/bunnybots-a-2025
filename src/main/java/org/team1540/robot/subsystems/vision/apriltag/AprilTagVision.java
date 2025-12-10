package org.team1540.robot.subsystems.vision.apriltag;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.team1540.robot.Constants;
import org.team1540.robot.RobotState;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagVisionIO frontLeftCameraIO;
    private final AprilTagVisionIO frontRightCameraIO;
    private final PhotonVisionIOInputsAutoLogged frontRightCameraInputs = new PhotonVisionIOInputsAutoLogged();
    private final PhotonVisionIOInputsAutoLogged frontLeftCameraInputs = new PhotonVisionIOInputsAutoLogged();

    public AprilTagVision() {
        frontLeftCameraIO =
                new AprilTagVisionIOPhoton("left", AprilTagVisionConstants.cameraOffsets.frontLeftCameraPLaceOnRobot);
        frontRightCameraIO =
                new AprilTagVisionIOPhoton("right", AprilTagVisionConstants.cameraOffsets.frontRightCameraPLaceOnRobot);
    }

    public AprilTagVision(AprilTagVisionIO frontLeftCameraIO, AprilTagVisionIO frontRightCameraIO) {
        this.frontLeftCameraIO = frontLeftCameraIO;
        this.frontRightCameraIO = frontRightCameraIO;
    }

    public static AprilTagVision createReal() {
        if (Constants.CURRENT_MODE== Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real vision on simulated robot", false);
        }
        return new AprilTagVision(
                new AprilTagVisionIOPhoton(
                        "left simulated camera", AprilTagVisionConstants.cameraOffsets.frontLeftCameraPLaceOnRobot),
                new AprilTagVisionIOPhoton(
                        "right simulated camera", AprilTagVisionConstants.cameraOffsets.frontRightCameraPLaceOnRobot));
    }

    public static AprilTagVision createDummy() {
        if (Constants.CURRENT_MODE== Constants.Mode.SIM) {
            DriverStation.reportWarning("Using dummy vision on real robot", false);
        }
        return new AprilTagVision();
    }

    private AprilTagVisionIO.PoseObservation findLatest() {
        double delta = Double.POSITIVE_INFINITY;
        AprilTagVisionIOPhoton tl = (AprilTagVisionIOPhoton) frontLeftCameraIO;
        AprilTagVisionIOPhoton tr = (AprilTagVisionIOPhoton) frontRightCameraIO;
        AprilTagVisionIOPhoton best = null;

        if (tl.getLastTimeStamp() - System.currentTimeMillis() < delta) {
            delta = tl.getLastTimeStamp();
            best = tl;
        }
        if (tr.getLastTimeStamp() - System.currentTimeMillis() < delta) {
            //            delta = tr.getLastTimeStamp();
            best = tr;
        }
        if (best != null) {
            return best.getLast();
        } else {
            return null;
        }
    }

    @Override
    public void periodic() {
        AprilTagVisionIOPhoton frontLeftCamera = (AprilTagVisionIOPhoton) frontLeftCameraIO;
        AprilTagVisionIOPhoton frontRightCamera = (AprilTagVisionIOPhoton) frontRightCameraIO;
        frontLeftCameraIO.updateInputs(frontLeftCameraInputs);
        frontRightCameraIO.updateInputs(frontRightCameraInputs);
        Logger.processInputs("Vision", (LoggableInputs) frontRightCameraInputs);
        Logger.processInputs("Vision", (LoggableInputs) frontLeftCameraInputs);
        if (frontLeftCamera.getLast() != null) {
            RobotState.getInstance().addVisionMeasurement(frontLeftCamera.getLast());
        }
        if (frontRightCamera.getLast() != null) {
            RobotState.getInstance().addVisionMeasurement(frontRightCamera.getLast());
        }
    }
}
