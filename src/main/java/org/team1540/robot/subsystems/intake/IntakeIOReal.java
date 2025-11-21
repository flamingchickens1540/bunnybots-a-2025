package org.team1540.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import static org.team1540.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX rollerFalcon = new TalonFX(ROLLER_MOTOR_ID);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    private final StatusSignal<AngularVelocity> rollerVelocity = rollerFalcon.getVelocity();
    private final StatusSignal<Angle> rollerPosition = rollerFalcon.getPosition();
    private final StatusSignal<Voltage> rollerAppliedVoltage = rollerFalcon.getMotorVoltage();
    private final StatusSignal<Current> rollerSupplyCurrent = rollerFalcon.getSupplyCurrent();
    private final StatusSignal<Current> rollerStatorCurrent = rollerFalcon.getStatorCurrent();
    private final StatusSignal<Temperature> rollerTemp = rollerFalcon.getDeviceTemp();

    private final TalonFX pivotFalcon = new TalonFX(PIVOT_MOTOR_ID);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final StatusSignal<AngularVelocity> pivotVelocity = pivotFalcon.getVelocity();
    private final StatusSignal<Angle> pivotPosition = pivotFalcon.getPosition();
    private final StatusSignal<Voltage> pivotAppliedVoltage = pivotFalcon.getMotorVoltage();
    private final StatusSignal<Current> pivotSupplyCurrent = pivotFalcon.getSupplyCurrent();
    private final StatusSignal<Current> pivotStatorCurrent = pivotFalcon.getStatorCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivotFalcon.getDeviceTemp();



    public IntakeIOReal() {
        TalonFXConfiguration rollerTalonFXConfigs = new TalonFXConfiguration();
        TalonFXConfiguration pivotTalonFXConfigs = new TalonFXConfiguration();

        rollerTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        MotionMagicConfigs motionMagicConfigs = pivotTalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = PIVOT_ACCELERATION_RPS2;
        motionMagicConfigs.MotionMagicCruiseVelocity = PIVOT_CRUISE_VELOCITY_RPS;
    }


    @Override
    public void setPivotSetpoint(Rotation2d rotations) {
    }

    @Override
    public void resetPivotPosition(Rotation2d rotations) {
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotFalcon.setControl(pivotVoltageRequest.withOutput(voltage));
    }


    @Override
    public void setRollerVoltage(double voltage) {
        rollerFalcon.setControl(rollerVoltageRequest.withOutput(voltage));
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        StatusCode rollerStatus = BaseStatusSignal.refreshAll (
            rollerVelocity, rollerPosition, rollerAppliedVoltage, rollerSupplyCurrent, rollerStatorCurrent, rollerTemp
        );

        StatusCode pivotStatus = BaseStatusSignal.refreshAll (
            pivotVelocity, pivotPosition, pivotAppliedVoltage, pivotSupplyCurrent, pivotStatorCurrent, pivotTemp
        );

        inputs.rollerMotorVelocityRPS = rollerVelocity.getValueAsDouble();
        inputs.rollerMotorAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerStatorCurrentAmps = rollerStatorCurrent.getValueAsDouble();

        inputs.pivotMotorVelocityRPS = pivotVelocity.getValueAsDouble();
        inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValueAsDouble());
        inputs.pivotMotorAppliedVolts = pivotAppliedVoltage.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = pivotStatorCurrent.getValueAsDouble();

    }

    public void setPivotPID(double kP, double kI, double kD) {
    }

    public void setPivotFF(double kS, double kV, double kG) {
    }
}

// IntakeIO interface - done
// IntakeConstants class
// IntakeIO implementation
// Intake class

// Write updateInputs
// Learn about setPivotPID and setPivotFF
// write setPivotSetpoint and resetPivotPosition
// Finish constructor
// Write fields

// Ask design how indexer handoff works
// Go through code (and search up questions/Slack)