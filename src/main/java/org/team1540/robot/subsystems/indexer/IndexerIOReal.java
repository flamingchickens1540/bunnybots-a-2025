package org.team1540.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerIOReal implements IndexerIO {

    private final TalonFX rightMotor = new TalonFX(RIGHT_MOTOR_ID);
    private final TalonFX upperMotor = new TalonFX(UPPER_MOTOR_ID);

    private final MotionMagicVoltage rightMM = new MotionMagicVoltage(0).withEnableFOC(true);
    private final MotionMagicVoltage upperMM = new MotionMagicVoltage(0).withEnableFOC(true);

    public IndexerIOReal() {
        TalonFXConfiguration rightCfg = new TalonFXConfiguration();
        rightCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightCfg.Feedback.SensorToMechanismRatio = RIGHT_GEAR_RATIO;
        rightCfg.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT;
        rightCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        rightCfg.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT;
        rightCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightCfg.Slot0.kP = kP;
        rightCfg.Slot0.kI = kI;
        rightCfg.Slot0.kD = kD;
        rightCfg.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        rightCfg.MotionMagic.MotionMagicAcceleration = ACCELERATION_RPS2;

        TalonFXConfiguration upperCfg = new TalonFXConfiguration();
        upperCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        upperCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        upperCfg.Feedback.SensorToMechanismRatio = UPPER_GEAR_RATIO;
        upperCfg.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT;
        upperCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        upperCfg.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT;
        upperCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        upperCfg.Slot0.kP = kP;
        upperCfg.Slot0.kI = kI;
        upperCfg.Slot0.kD = kD;
        upperCfg.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        upperCfg.MotionMagic.MotionMagicAcceleration = ACCELERATION_RPS2;

        rightMotor.getConfigurator().apply(rightCfg);
        upperMotor.getConfigurator().apply(upperCfg);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.rightVelocityRPS = rightMotor.getVelocity().getValueAsDouble();
        inputs.upperVelocityRPS = upperMotor.getVelocity().getValueAsDouble();
        inputs.rightPositionRot = rightMotor.getPosition().getValueAsDouble();
        inputs.upperPositionRot = upperMotor.getPosition().getValueAsDouble();
        inputs.rightCurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
        inputs.upperCurrentAmps = upperMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setRightMMPosition(double rotations) {
        rightMotor.setControl(rightMM.withPosition(rotations));
    }

    @Override
    public void setUpperMMPosition(double rotations) {
        upperMotor.setControl(upperMM.withPosition(rotations));
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
        upperMotor.stopMotor();
    }
}
