package org.team1540.robot.subsystems.intake;

import static org.team1540.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static boolean hasInstance = false;

    // public enum IntakeState {
    //     //add the states

    //     private final DoubleSupplier pivotPosition;

    //     IntakeState(DoubleSupplier pivotPositionDeg) {
    //         this.pivotPosition = pivotPositionDeg;
    //     }

    //     public Rotation2d pivotPosition() {
    //         return Rotation2d.fromDegrees(pivotPosition.getAsDouble());
    //     }
    // }

    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    // private final LoggedTunableNumber pivotKP = new LoggedTunableNumber("Intake/kP", PIVOT_KP);
    // private final LoggedTunableNumber pivotKI = new LoggedTunableNumber("Intake/kI", PIVOT_KI);
    // private final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Intake/kD", PIVOT_KD);
    // private final LoggedTunableNumber pivotKS = new LoggedTunableNumber("Intake/kS", PIVOT_KS);
    // private final LoggedTunableNumber pivotKV = new LoggedTunableNumber("Intake/kV", PIVOT_KV);
    // private final LoggedTunableNumber pivotKG = new LoggedTunableNumber("Intake/kG", PIVOT_KG);

    private Rotation2d pivotSetpoint = PIVOT_MIN_ANGLE;

    private Intake(IntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            stopAll();
        }

        LoggedTracer.reset();

        io.updateInputs(inputs);
    }

    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public void setPivotPosition(Rotation2d rotations) {
        pivotSetpoint = rotations;
        io.setPivotSetpoint(rotations);
    }

    public Rotation2d getPivotPosition() {
        return inputs.pivotPosition;
    }

    public void setPivotVoltage(double voltage) {
        io.setPivotVoltage(voltage);
    }

    public void stopAll() {
        setRollerVoltage(0);
        setPivotVoltage(0);
    }

    public void holdPivot() {
        setPivotPosition(inputs.pivotPosition);
    }
}
