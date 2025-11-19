package org.team1540.robot.subsystems.vision.apriltag;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.team1540.robot.RobotState;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagVisionIO iotl = new AprilTagVisionIOPhoton("tl", AprilTagVisionConstants.cameraOffsets.tl);
    private final AprilTagVisionIO iotr = new AprilTagVisionIOPhoton("tl", AprilTagVisionConstants.cameraOffsets.tr);
    private final AprilTagVisionIO iobl = new AprilTagVisionIOPhoton("tl", AprilTagVisionConstants.cameraOffsets.bl);
    private final AprilTagVisionIO iobr = new AprilTagVisionIOPhoton("tl", AprilTagVisionConstants.cameraOffsets.bl);
    private final AprilTagVisionIO.PhotonVisionIOInputs inputs = new AprilTagVisionIO.PhotonVisionIOInputs();

    public AprilTagVision() {}

    public static AprilTagVision createReal() {
        if (!AprilTagVisionConstants.CONTROL_MODE) {
            DriverStation.reportWarning("Using real vision on simulated robot", false);
        }
        return new AprilTagVision();
    }

    public static AprilTagVision createDummy() {
        if (!AprilTagVisionConstants.CONTROL_MODE) {
            DriverStation.reportWarning("Using dummy vision on real robot", false);
        }
        return new AprilTagVision();
    }

    private AprilTagVisionIO.PoseObservation findLatest() {
        double delta = Double.POSITIVE_INFINITY;
        AprilTagVisionIOPhoton tl = (AprilTagVisionIOPhoton) iotl;
        AprilTagVisionIOPhoton tr = (AprilTagVisionIOPhoton) iotr;
        AprilTagVisionIOPhoton bl = (AprilTagVisionIOPhoton) iobl;
        AprilTagVisionIOPhoton br = (AprilTagVisionIOPhoton) iobr;
        AprilTagVisionIOPhoton best = null;

        if (tl.getLastTimeStamp() - System.currentTimeMillis() < delta) {
            delta = tl.getLastTimeStamp();
            best = tl;
        }
        if (tr.getLastTimeStamp() - System.currentTimeMillis() < delta) {
            delta = tr.getLastTimeStamp();
            best = tr;
        }
        if (bl.getLastTimeStamp() - System.currentTimeMillis() < delta) {
            delta = bl.getLastTimeStamp();
            best = bl;
        }
        if (br.getLastTimeStamp() - System.currentTimeMillis() < delta) {
            delta = br.getLastTimeStamp();
            best = br;
        }
        if (best != null) {
            return best.getLast();
        } else {
            return null;
        }
    }

    @Override
    public void periodic() {
        iotl.updateInputs(inputs);
        iotr.updateInputs(inputs);
        iobl.updateInputs(inputs);
        iobr.updateInputs(inputs);
        Logger.processInputs("Vision", (LoggableInputs) inputs);
        if (findLatest() != null) {
            RobotState.getInstance().addVisionMeasurement(findLatest());
        }
    }
}
