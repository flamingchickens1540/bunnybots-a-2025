package org.team1540.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public double rightVelocityRPS;
        public double rightSupplyCurrentAmps;
        public double rightStatorCurrentAmps;
        public double rightAppliedVolts;
        public double rightMotorTemp;

        public double upperVelocityRPS;
        public double upperSupplyCurrentAmps;
        public double upperStatorCurrentAmps;
        public double upperAppliedVolts;
        public double upperMotorTemp;

        public boolean sensorConnected;
        public boolean sensorTripped;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setRightVoltage(double volts) {}

    default void setUpperVoltage(double volts) {}

    default void stopAll() {}
}
