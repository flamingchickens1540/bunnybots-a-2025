package org.team1540.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    class IntakeInputs {
        public boolean rollerConnected = true;
        public double rollerMotorVelocityRPS = 0;
        public double rollerMotorAppliedVolts = 0;
        public double rollerSupplyCurrentAmps = 0;
        public double rollerStatorCurrentAmps = 0;

        public boolean pivotConnected = true;
        public Rotation2d pivotPosition = Rotation2d.kZero;
        public double pivotMotorVelocityRPS = 0;
        public double pivotMotorAppliedVolts = 0;
        public double pivotSupplyCurrentAmps = 0;
        public double pivotStatorCurrentAmps = 0;
    }

    default void setRollerVoltage(double voltage) {}

    default void setPivotSetpoint(Rotation2d rotations) {}

    default void resetPivotPosition(Rotation2d rotations) {}

    default void setPivotVoltage(double voltage) {}

    default void setPivotPID(double kP, double kI, double kD) {}

    default void setPivotFF(double kS, double kV, double kG) {}

    default void updateInputs(IntakeInputs inputs) {}
}
