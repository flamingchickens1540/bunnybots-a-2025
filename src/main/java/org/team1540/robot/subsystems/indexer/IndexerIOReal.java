package org.team1540.robot.subsystems.indexer;

import static org.team1540.robot.subsystems.indexer.IndexerConstants.*;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOReal implements IndexerIO {

    private final TalonFX rightMotor = new TalonFX(RIGHT_MOTOR_ID);
    private final TalonFX upperMotor = new TalonFX(UPPER_MOTOR_ID);

    private final LaserCan laserCan = new LaserCan(LASER_CAN_ID);

    private final StatusSignal<Voltage> rightAppliedVoltage = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> rightSupplyCurrent = rightMotor.getSupplyCurrent();
    private final StatusSignal<Current> rightStatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Temperature> rightTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();

    private final StatusSignal<Voltage> upperAppliedVoltage = upperMotor.getMotorVoltage();
    private final StatusSignal<Current> upperSupplyCurrent = upperMotor.getSupplyCurrent();
    private final StatusSignal<Current> upperStatorCurrent = upperMotor.getStatorCurrent();
    private final StatusSignal<Temperature> upperTemp = upperMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> upperVelocity = upperMotor.getVelocity();

    private final MotionMagicVoltage rightMM = new MotionMagicVoltage(0).withEnableFOC(true);
    private final MotionMagicVoltage upperMM = new MotionMagicVoltage(0).withEnableFOC(true);

    public IndexerIOReal() {
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.Feedback.SensorToMechanismRatio = RIGHT_GEAR_RATIO;
        rightMotorConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT;
        rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMotorConfig.Slot0.kP = kP;
        rightMotorConfig.Slot0.kI = kI;
        rightMotorConfig.Slot0.kD = kD;
        rightMotorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        rightMotorConfig.MotionMagic.MotionMagicAcceleration = ACCELERATION_RPS2;

        TalonFXConfiguration upperMotorConfig = new TalonFXConfiguration();
        upperMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        upperMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        upperMotorConfig.Feedback.SensorToMechanismRatio = UPPER_GEAR_RATIO;
        upperMotorConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT;
        upperMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        upperMotorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT;
        upperMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        upperMotorConfig.Slot0.kP = kP;
        upperMotorConfig.Slot0.kI = kI;
        upperMotorConfig.Slot0.kD = kD;
        upperMotorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        upperMotorConfig.MotionMagic.MotionMagicAcceleration = ACCELERATION_RPS2;

        rightMotor.getConfigurator().apply(rightMotorConfig);
        upperMotor.getConfigurator().apply(upperMotorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightStatorCurrent,
                rightTemp,
                rightVelocity,
                upperAppliedVoltage,
                upperSupplyCurrent,
                upperStatorCurrent,
                upperTemp,
                upperVelocity);

        rightMotor.optimizeBusUtilization();
        upperMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightStatorCurrent,
                rightTemp,
                rightVelocity,
                upperAppliedVoltage,
                upperSupplyCurrent,
                upperStatorCurrent,
                upperTemp,
                upperVelocity);

        inputs.rightVelocityRPS = rightVelocity.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
        inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
        inputs.rightAppliedVolts = upperAppliedVoltage.getValueAsDouble();
        inputs.rightMotorTemp = rightTemp.getValueAsDouble();

        inputs.upperVelocityRPS = upperVelocity.getValueAsDouble();
        inputs.upperStatorCurrentAmps = upperStatorCurrent.getValueAsDouble();
        inputs.upperSupplyCurrentAmps = upperSupplyCurrent.getValueAsDouble();
        inputs.upperAppliedVolts = upperAppliedVoltage.getValueAsDouble();
        inputs.upperMotorTemp = upperTemp.getValueAsDouble();

        LaserCanInterface.Measurement measurement = laserCan.getMeasurement();
        inputs.sensorTripped =
                measurement != null
                && measurement.distance_mm <= LASER_CAN_DETECTION_DISTANCE_MM;

        inputs.sensorConnected = measurement != null;
    }

    @Override
    public void setRightVoltage(double volts) {
        rightMotor.setVoltage(volts);
    }

    @Override
    public void setUpperVoltage(double volts) {
        upperMotor.setVoltage(volts);
    }

    @Override
    public void stopAll() {
        rightMotor.stopMotor();
        upperMotor.stopMotor();
    }
}
