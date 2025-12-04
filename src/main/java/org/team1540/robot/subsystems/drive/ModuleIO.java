package org.team1540.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveIsConnected = false;
        public double drivePositionRads = 0;
        public double driveVolts = 0;
        public double driveSupplyCurrent = 0;
        public double driveStatorCurrent = 0;
        public double driveVelocityRadsPerSec = 0;
        public double driveTempC = 0;

        public boolean turnIsConnected = false;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public double turnVolts = 0;
        public double turnSupplyCurrent = 0;
        public double turnStatorCurrent = 0;
        public double turnVelocityRadsPerSec = 0;
        public double turnTempC = 0;

        public Rotation2d encoderPosition = Rotation2d.kZero;
        public boolean encoderIsConnected = false;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRads = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    default void updateInputs(ModuleIOInputs inputs) {}

    default void setDriveVelocity(double driveVelocityRadsPerSec) {}

    default void setDriveVolts(double driveMotorVolts) {}

    default void setTurnPosition(Rotation2d turnMotorPosition) {}

    default void setTurnVolts(double turnMotorVolts) {}

    default void setBrakeMode(boolean enabled) {}
}
