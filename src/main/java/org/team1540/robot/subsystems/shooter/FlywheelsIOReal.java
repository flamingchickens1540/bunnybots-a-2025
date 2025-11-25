package org.team1540.robot.subsystems.shooter;

import static org.team1540.robot.subsystems.shooter.ShooterConstants.Flywheels.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelsIOReal implements FlywheelsIO {

    private final TalonFX leftMotor = new TalonFX(LEFT_ID);
    private final TalonFX rightMotor = new TalonFX(RIGHT_ID);

    private final StatusSignal<AngularVelocity> leftVelocity = leftMotor.getVelocity();
    private final StatusSignal<Voltage> leftAppliedVolts = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> leftCurrent = leftMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> leftTempCelsius = leftMotor.getDeviceTemp();

    private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();
    private final StatusSignal<Voltage> rightAppliedVolts = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> rightCurrent = rightMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> rightTempCelsius = rightMotor.getDeviceTemp();

    private final VelocityVoltage leftVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut leftVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    private final VelocityVoltage rightVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut rightVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    // Config ):
    public FlywheelsIOReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // TODO: finsh config with voltage limits and feedback values

        // Set PID and feed forward values
        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;

        // Sets neutral mode (no input) to coast
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Apply the configuration to the left motor
        leftMotor.getConfigurator().apply(config);

        // Set inverted and applys config to right motor
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(config);

        // Sets update freqency for the listed values while disabling any other status signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leftVelocity,
                leftAppliedVolts,
                leftCurrent,
                leftTempCelsius,
                rightVelocity,
                rightAppliedVolts,
                rightCurrent,
                rightTempCelsius);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftVelocity,
                leftAppliedVolts,
                leftCurrent,
                leftTempCelsius,
                rightVelocity,
                rightAppliedVolts,
                rightCurrent,
                rightTempCelsius);

        inputs.leftVelocityRPM = leftVelocity.getValueAsDouble() * 60;
        inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();

        inputs.rightVelocityRPM = rightVelocity.getValueAsDouble() * 60;
        inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
    }

    @Override
    public void setSpeeds(double leftRPM, double rightRPM) {
        leftMotor.setControl(leftVelocityCtrlReq.withVelocity(leftRPM / 60));
        rightMotor.setControl(rightVelocityCtrlReq.withVelocity(rightRPM / 60));
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftMotor.setControl(leftVoltageCtrlReq.withOutput(leftVolts));
        rightMotor.setControl(rightVoltageCtrlReq.withOutput(rightVolts));
    }

    // function for PID tunning
    @Override
    public void configPID(double kP, double kI, double kD, double kV) {
        Slot0Configs pidConfigs = new Slot0Configs();
        leftMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pidConfigs.kV = kV;
        leftMotor.getConfigurator().apply(pidConfigs);
        rightMotor.getConfigurator().apply(pidConfigs);
    }
}
