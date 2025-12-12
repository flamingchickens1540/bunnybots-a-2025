package org.team1540.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

    @AutoLog
    class FeederIOInputs {
        public double feederAppliedVolts = 0.0;
        public double feederCurrentAmps = 0.0;
        public double feederVelocityRPM = 0.0;
        public double feederTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(FeederIOInputs inputs) {}

    /**
     * Runs open loop at the specified voltages
     */
    default void setVoltage(double volts) {}

    /**
     * Runs closed loop at the specified RPMs
     */
}
