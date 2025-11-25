package org.team1540.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.dyn4j.geometry.Rotatable;

import static org.team1540.robot.subsystems.shooter.ShooterConstants.Pivot.*;

public class ShooterPivotIOReal implements ShooterPivotIO {

    private final TalonFX motor = new TalonFX(MOTOR_ID);

    private final StatusSignal<Angle> position = motor.getPosition();
    // check if it's an angle or double
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<Current> suppliedCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> temp = motor.getDeviceTemp();
    private final StatusSignal<ForwardLimitValue> forwardLimit = motor.getForwardLimit();
    private final StatusSignal<ReverseLimitValue> reverseLimit = motor.getReverseLimit();

    private final MotionMagicVoltage positionCtrlRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageCtrlRequest = new VoltageOut(0);

    public ShooterPivotIOReal(){
        TalonFXConfiguration motorConfig=new TalonFXConfiguration();
        //todo : see if its inverted
        motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //todo: see if its inverted


        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE;

        motorConfig.Slot0.kP = KP;
        motorConfig.Slot0.kI = KI;
        motorConfig.Slot0.kD = KD;
        motorConfig.Slot0.kS = KS;
        motorConfig.Slot0.kG = KG;
        motorConfig.Slot0.kV = KV;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCEL_RPS2;
        motorConfig.MotionMagic.MotionMagicJerk = JERK_RPS3;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT;

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                velocity,
                suppliedCurrent,
                appliedVoltage,
                temp,
                forwardLimit,
                reverseLimit);

        motor.optimizeBusUtilization();
    }
    @Override
    public void updateInputs(ShooterPivotIOInputs inputs){
        BaseStatusSignal.refreshAll(position, velocity,suppliedCurrent,appliedVoltage,temp,forwardLimit,reverseLimit);
        inputs.isAtForwardLimit = forwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
        inputs.isAtReverseLimit = reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.position = (edu.wpi.first.math.geometry.Rotation2d) position.getValue();
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.appliedVolts= appliedVoltage.getValueAsDouble();
        inputs.currentAmps = suppliedCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void setPosition(Rotation2d position){
        motor.setControl(positionCtrlRequest.withPosition(position.getRotations()));
    }

    @Override
    public void setVoltage(double volts){
        motor.setControl(voltageCtrlRequest.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode){
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kG){
        Slot0Configs pidConfigs=new Slot0Configs();
        motor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pidConfigs.kG = kG;
        motor.getConfigurator().apply(pidConfigs);
    }

    @Override
    public void setEncoderPosition(double rotations){
        motor.setPosition(rotations);
    }

}
