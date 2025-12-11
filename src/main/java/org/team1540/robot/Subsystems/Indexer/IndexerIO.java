package org.team1540.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public double rightVelocityRPS;
        public double upperVelocityRPS;
        public double rightPositionRot;
        public double upperPositionRot;
        public double rightCurrentAmps;
        public double upperCurrentAmps;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setRightMMPosition(double rotations) {}

    default void setUpperMMPosition(double rotations) {}

    default void stop() {}
}
