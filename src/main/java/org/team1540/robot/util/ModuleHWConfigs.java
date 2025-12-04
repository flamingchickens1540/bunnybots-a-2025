package org.team1540.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public record ModuleHWConfigs(
        TalonFXConfiguration driveConfig, TalonFXConfiguration turnConfig, CANcoderConfiguration canCoderConfig) {
    public static ModuleHWConfigs fromModuleConstants(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Slot0 = constants.DriveMotorGains;

        TalonFXConfiguration turnConfig = constants.SteerMotorInitialConfigs;
        turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.SensorToMechanismRatio = 1.0;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.Slot0 = constants.SteerMotorGains;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        CANcoderConfiguration turnEncoderConfig = constants.EncoderInitialConfigs;
        turnEncoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        turnEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turnEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        return new ModuleHWConfigs(driveConfig, turnConfig, turnEncoderConfig);
    }
}
