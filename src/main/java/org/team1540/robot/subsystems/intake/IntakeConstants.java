package org.team1540.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class IntakeConstants {
    // placeholder values for now

    public static final int ROLLER_MOTOR_ID = 0;
    public static final int PIVOT_MOTOR_ID = 0;
    public static final int PIVOT_CRUISE_VELOCITY_RPS = 0;
    public static final int PIVOT_ACCELERATION_RPS2 = 0;

    public static final Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(0);

    public static final Translation3d ROTATIONAL_ORIGIN = new Translation3d(0.304800, 0.0, 0.1270000);

    public static final double PIVOT_KP = 0;
    public static final double PIVOT_KI = 0;
    public static final double PIVOT_KD = 0;
    public static final double PIVOT_KS = 0;
    public static final double PIVOT_KV = 0;
    public static final double PIVOT_KG = 0;

    public static final double PIVOT_GEAR_RATIO = 1;
    public static final double ROLLER_GEAR_RATIO = 1;
}
