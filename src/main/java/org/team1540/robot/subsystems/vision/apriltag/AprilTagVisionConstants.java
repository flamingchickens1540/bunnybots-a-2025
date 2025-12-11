package org.team1540.robot.subsystems.vision.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public class AprilTagVisionConstants {
    public static final double MAX_ROBOT_Z_TOLERANCE = 0.5;

    public static class FieldConstants {
        public static AprilTagFieldLayout aprilTagLayout;

        static {
            try {
                aprilTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory()
                        .toPath()
                        .resolve("2025-bunnybots.json")
                        .toString());
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    static class cameraOffsets {
        public static Transform3d frontRightCameraPLaceOnRobot = new Transform3d();
        public static Transform3d frontLeftCameraPLaceOnRobot = new Transform3d();
    }
}
