package org.team1540.robot.subsystems.vision.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.team1540.robot.Robot;

public class AprilTagVisionConstants {
    public static final boolean CONTROL_MODE = Robot.isReal();
    public static final double MAX_ROBOT_Z_TOLERANCE = 0.5;

    static class MODE {}

    public static class FieldConstants {
        public static AprilTagFieldLayout aprilTagLayout;
    }

    static class cameraOffsets {
        public static Transform3d tr;
        public static Transform3d tl;
        public static Transform3d br;
        public static Transform3d bl;
    }
}
