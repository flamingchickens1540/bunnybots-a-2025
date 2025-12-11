package org.team1540.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean isConnected = false;
        public double yawVelocityRadsPerSec = 0;
        public Rotation2d yawPosition = Rotation2d.kZero;

        public double[] odometryTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    default void updateInputs(GyroIOInputs inputs) {}
}
