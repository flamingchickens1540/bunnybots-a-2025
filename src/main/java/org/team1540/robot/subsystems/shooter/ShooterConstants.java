package org.team1540.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {

    public static class Flywheels {
        public static final int TOP_ID = 13;
        public static final int BOTTOM_ID = 6;

        public static final double KP = 0.3;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.20647629357556074;
        public static final double KV = 0.12065235202423787;
    }

    public static class Pivot {
        public static final int MOTOR_ID = 15;
        public static final double KP = 670;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KG = 0.0;

        public static final double PIVOT_GEAR_RATIO = (50.0 / 12.0) * (50.0 / 20) * 6;

        public static final Rotation2d MAX_ANGLE_ROTS = Rotation2d.kCCW_90deg;
        public static final Rotation2d MIN_ANGLE_ROTS = Rotation2d.fromDegrees(57);
        public static final double CRUISE_VELOCITY_RPS = 2.0;
        public static final double MAX_ACCEL_RPS2 = 20.0;
        public static final double JERK_RPS3 = 200.0;

        public static final double SUPPLY_CURRENT_LIMIT = 10;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 0.1;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 15;
        public static final double ERROR_TOLERANCE_ROTS = 67;

        public static final double HARD_STOP_CURRENT = 41;
    }

    public static class Feeder {
        public static final int FEEDER_ID = 67;
        public static final int LASER_CAN_ID = 67;
    }
}
