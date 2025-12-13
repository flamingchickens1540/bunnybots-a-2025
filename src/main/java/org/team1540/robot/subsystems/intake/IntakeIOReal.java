package org.team1540.robot.subsystems.intake;

import static org.team1540.robot.subsystems.intake.IntakeConstants.*;

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
import org.team1540.robot.generated.TunerConstants;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX rollerFalcon = new TalonFX(ROLLER_MOTOR_ID);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    private final StatusSignal<AngularVelocity> rollerVelocity = rollerFalcon.getVelocity();
    private final StatusSignal<Angle> rollerPosition = rollerFalcon.getPosition();
    private final StatusSignal<Voltage> rollerAppliedVoltage = rollerFalcon.getMotorVoltage();
    private final StatusSignal<Current> rollerSupplyCurrent = rollerFalcon.getSupplyCurrent();
    private final StatusSignal<Current> rollerStatorCurrent = rollerFalcon.getStatorCurrent();
    private final StatusSignal<Temperature> rollerTemp = rollerFalcon.getDeviceTemp();

    private final TalonFX pivotFalcon = new TalonFX(PIVOT_MOTOR_ID, TunerConstants.kCANBus);
    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final StatusSignal<AngularVelocity> pivotVelocity = pivotFalcon.getVelocity();
    private final StatusSignal<Angle> pivotPosition = pivotFalcon.getPosition();
    private final StatusSignal<Voltage> pivotAppliedVoltage = pivotFalcon.getMotorVoltage();
    private final StatusSignal<Current> pivotSupplyCurrent = pivotFalcon.getSupplyCurrent();
    private final StatusSignal<Current> pivotStatorCurrent = pivotFalcon.getStatorCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivotFalcon.getDeviceTemp();

    private final Debouncer rollerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer pivotConnectedDebounce = new Debouncer(0.5);

    public IntakeIOReal() {
        TalonFXConfiguration rollerTalonFXConfigs = new TalonFXConfiguration();
        TalonFXConfiguration pivotTalonFXConfigs = new TalonFXConfiguration();

        // copied from last year's code
        rollerTalonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        rollerTalonFXConfigs.CurrentLimits.withStatorCurrentLimit(120);
        rollerTalonFXConfigs.CurrentLimits.withSupplyCurrentLimit(55);
        rollerTalonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerTalonFXConfigs.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;

        pivotTalonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        pivotTalonFXConfigs.CurrentLimits.withStatorCurrentLimit(120);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLimit(70);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLowerLimit(40);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLowerTime(0.5);
        pivotTalonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotTalonFXConfigs.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;

        rollerTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs pivotGains = pivotTalonFXConfigs.Slot0;
        pivotGains.kS = PIVOT_KS;
        pivotGains.kV = PIVOT_KV;
        pivotGains.kG = PIVOT_KG;
        pivotGains.kP = PIVOT_KP;
        pivotGains.kI = PIVOT_KI;
        pivotGains.kD = PIVOT_KD;
        pivotGains.GravityType = GravityTypeValue.Arm_Cosine;

        MotionMagicConfigs motionMagicConfigs = pivotTalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = PIVOT_ACCELERATION_RPS2;
        motionMagicConfigs.MotionMagicCruiseVelocity = PIVOT_CRUISE_VELOCITY_RPS;

        pivotFalcon.getConfigurator().apply(pivotTalonFXConfigs);
        rollerFalcon.getConfigurator().apply(pivotTalonFXConfigs);

        pivotFalcon.getConfigurator().apply(pivotTalonFXConfigs);
        pivotFalcon.setPosition(PIVOT_MAX_ANGLE.getRotations());

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                rollerVelocity,
                rollerPosition,
                rollerAppliedVoltage,
                rollerSupplyCurrent,
                rollerStatorCurrent,
                rollerTemp,
                pivotVelocity,
                pivotPosition,
                pivotAppliedVoltage,
                pivotSupplyCurrent,
                pivotStatorCurrent,
                pivotTemp);
        rollerFalcon.optimizeBusUtilization();
        pivotFalcon.optimizeBusUtilization();
    }

    @Override
    public void setPivotSetpoint(Rotation2d rotations) {
        pivotFalcon.setControl(pivotPositionRequest.withPosition(rotations.getRotations()));
    }

    @Override
    public void resetPivotPosition(Rotation2d rotations) {
        pivotFalcon.setPosition(rotations.getRotations());
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
        StatusCode rollerStatus = BaseStatusSignal.refreshAll(
                rollerVelocity,
                rollerPosition,
                rollerAppliedVoltage,
                rollerSupplyCurrent,
                rollerStatorCurrent,
                rollerTemp);

        StatusCode pivotStatus = BaseStatusSignal.refreshAll(
                pivotVelocity, pivotPosition, pivotAppliedVoltage, pivotSupplyCurrent, pivotStatorCurrent, pivotTemp);

        inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
        inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());

        inputs.rollerMotorVelocityRPS = rollerVelocity.getValueAsDouble();
        inputs.rollerMotorAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerStatorCurrentAmps = rollerStatorCurrent.getValueAsDouble();
        inputs.rollerMotorTemp = rollerTemp.getValueAsDouble();

        inputs.pivotMotorVelocityRPS = pivotVelocity.getValueAsDouble();
        inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValueAsDouble());
        inputs.pivotMotorAppliedVolts = pivotAppliedVoltage.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = pivotStatorCurrent.getValueAsDouble();
        inputs.pivotMotorTemp = pivotTemp.getValueAsDouble();
    }

    public void setPivotPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        pivotFalcon.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        pivotFalcon.getConfigurator().apply(configs);
    }

    public void setPivotFF(double kS, double kV, double kG) {
        Slot0Configs configs = new Slot0Configs();
        pivotFalcon.getConfigurator().refresh(configs);
        configs.kS = kS;
        configs.kV = kV;
        configs.kA = kG;
        pivotFalcon.getConfigurator().apply(configs);
    }
}
