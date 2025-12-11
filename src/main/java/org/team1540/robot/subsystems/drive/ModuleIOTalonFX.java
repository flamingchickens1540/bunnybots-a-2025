package org.team1540.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.Queue;
import org.team1540.robot.util.ModuleHWConfigs;

public class ModuleIOTalonFX implements ModuleIO {
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;

    private final StatusSignal<Voltage> driveVolts;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<Current> driveSupplyCurrent;
    private final StatusSignal<Current> driveStatorCurrent;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Temperature> driveTemp;

    private final StatusSignal<Voltage> turnVolts;
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<Current> turnSupplyCurrent;
    private final StatusSignal<Current> turnStatorCurrent;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Temperature> turnTemp;

    private final StatusSignal<Angle> canCoderPosition;

    private final VoltageOut driveVoltReq = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage driveVelReq = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut turnVoltReq = new VoltageOut(0).withEnableFOC(true);
    private final PositionVoltage turnPosReq = new PositionVoltage(0).withEnableFOC(true);

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration turnConfig;

    public ModuleIOTalonFX(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        ModuleHWConfigs configs = ModuleHWConfigs.fromModuleConstants(constants);

        driveMotor = new TalonFX(constants.DriveMotorId);
        turnMotor = new TalonFX(constants.SteerMotorId);
        canCoder = new CANcoder(constants.EncoderId);

        driveConfig = configs.driveConfig();
        turnConfig = configs.turnConfig();
        driveMotor.getConfigurator().apply(driveConfig);
        turnMotor.getConfigurator().apply(turnConfig);
        canCoder.getConfigurator().apply(configs.canCoderConfig());

        driveVolts = driveMotor.getMotorVoltage();
        drivePosition = driveMotor.getPosition();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveStatorCurrent = driveMotor.getStatorCurrent();
        driveVelocity = driveMotor.getVelocity();
        driveTemp = driveMotor.getDeviceTemp();

        turnVolts = turnMotor.getMotorVoltage();
        turnPosition = turnMotor.getPosition();
        turnSupplyCurrent = turnMotor.getSupplyCurrent();
        turnStatorCurrent = turnMotor.getStatorCurrent();
        turnVelocity = turnMotor.getVelocity();
        turnTemp = turnMotor.getDeviceTemp();

        canCoderPosition = canCoder.getPosition();

        drivePositionQueue = OdometryThread.getInstance().registerSignal(drivePosition);
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnPosition);
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                driveVolts,
                driveStatorCurrent,
                driveSupplyCurrent,
                driveVelocity,
                driveTemp,
                turnVolts,
                turnSupplyCurrent,
                turnStatorCurrent,
                turnVelocity,
                turnTemp);
        BaseStatusSignal.setUpdateFrequencyForAll(
                DrivetrainConstants.ODOMETRY_FREQUENCY_HZ, drivePosition, turnPosition);

        driveMotor.optimizeBusUtilization();
        turnMotor.optimizeBusUtilization();
        canCoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                driveVolts,
                driveStatorCurrent,
                driveSupplyCurrent,
                driveVelocity,
                driveTemp,
                turnVolts,
                turnSupplyCurrent,
                turnStatorCurrent,
                turnVelocity,
                turnTemp,
                drivePosition,
                turnPosition);

        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        inputs.odometryDrivePositionsRads = drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();

        timestampQueue.clear();
        turnPositionQueue.clear();
        drivePositionQueue.clear();

        inputs.driveIsConnected = driveMotor.isConnected();
        inputs.driveVolts = driveVolts.getValueAsDouble();
        inputs.drivePositionRads = drivePosition.getValueAsDouble();
        inputs.driveSupplyCurrent = driveSupplyCurrent.getValueAsDouble();
        inputs.driveStatorCurrent = driveStatorCurrent.getValueAsDouble();
        inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveTempC = driveTemp.getValueAsDouble();

        inputs.turnIsConnected = turnMotor.isConnected();
        inputs.turnVolts = turnVolts.getValueAsDouble();
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnSupplyCurrent = turnSupplyCurrent.getValueAsDouble();
        inputs.turnStatorCurrent = turnStatorCurrent.getValueAsDouble();
        inputs.turnVelocityRadsPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnTempC = turnTemp.getValueAsDouble();

        inputs.encoderIsConnected = canCoder.isConnected();
        inputs.encoderPosition = Rotation2d.fromRotations(canCoderPosition.getValueAsDouble());
    }

    @Override
    public void setDriveVelocity(double driveVelocityRadsPerSec) {
        driveMotor.setControl(driveVelReq.withVelocity(Units.radiansToRotations(driveVelocityRadsPerSec)));
    }

    @Override
    public void setDriveVolts(double driveMotorVolts) {
        driveMotor.setControl(driveVoltReq.withOutput(driveMotorVolts));
    }

    @Override
    public void setTurnPosition(Rotation2d turnMotorPosition) {
        turnMotor.setControl(turnPosReq.withPosition(turnMotorPosition.getRotations()));
    }

    @Override
    public void setTurnVolts(double turnMotorVolts) {
        turnMotor.setControl(turnVoltReq.withOutput(turnMotorVolts));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        driveConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfig.MotorOutput);
        turnConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnMotor.getConfigurator().apply(turnConfig.MotorOutput);
    }
}
