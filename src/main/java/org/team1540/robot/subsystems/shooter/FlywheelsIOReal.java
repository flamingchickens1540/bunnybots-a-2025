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

    private final TalonFX topMotor = new TalonFX(TOP_ID);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_ID);

    private final StatusSignal<AngularVelocity> topVelocity = topMotor.getVelocity();
    private final StatusSignal<Voltage> topAppliedVolts = topMotor.getMotorVoltage();
    private final StatusSignal<Current> topCurrent = topMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> topTempCelsius = topMotor.getDeviceTemp();

    private final StatusSignal<AngularVelocity> bottomVelocity = bottomMotor.getVelocity();
    private final StatusSignal<Voltage> bottomAppliedVolts = bottomMotor.getMotorVoltage();
    private final StatusSignal<Current> bottomCurrent = bottomMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> bottomTempCelsius = bottomMotor.getDeviceTemp();

    private final VelocityVoltage topVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut topVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    private final VelocityVoltage bottomVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut bottomVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        topMotor.getConfigurator().apply(config);

        // Set inverted and applys config to right motor
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        bottomMotor.getConfigurator().apply(config);

        // Sets update freqency for the listed values while disabling any other status signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                topVelocity,
                topAppliedVolts,
                topCurrent,
                topTempCelsius,
                bottomVelocity,
                bottomAppliedVolts,
                bottomCurrent,
                bottomTempCelsius);

        topMotor.optimizeBusUtilization();
        bottomMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                topVelocity,
                topAppliedVolts,
                topCurrent,
                topTempCelsius,
                bottomVelocity,
                bottomAppliedVolts,
                bottomCurrent,
                bottomTempCelsius);

        inputs.topVelocityRPM = topVelocity.getValueAsDouble() * 60;
        inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
        inputs.topCurrentAmps = topCurrent.getValueAsDouble();
        inputs.topTempCelsius = topTempCelsius.getValueAsDouble();

        inputs.bottomVelocityRPM = bottomVelocity.getValueAsDouble() * 60;
        inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
        inputs.bottomCurrentAmps = bottomCurrent.getValueAsDouble();
        inputs.bottomTempCelsius = bottomTempCelsius.getValueAsDouble();
    }

    @Override
    public void setSpeeds(double topRPM, double bottomRPM) {
        topMotor.setControl(topVelocityCtrlReq.withVelocity(topRPM / 60));
        bottomMotor.setControl(bottomVelocityCtrlReq.withVelocity(bottomRPM / 60));
    }

    @Override
    public void setVoltage(double topVolts, double bottomVolts) {
        topMotor.setControl(topVoltageCtrlReq.withOutput(topVolts));
        bottomMotor.setControl(bottomVoltageCtrlReq.withOutput(bottomVolts));
    }

    // function for PID tunning
    @Override
    public void configPID(double kP, double kI, double kD, double kV, double kS) {
        Slot0Configs pidConfigs = new Slot0Configs();
        topMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pidConfigs.kV = kV;
        pidConfigs.kS = kS;
        topMotor.getConfigurator().apply(pidConfigs);
        bottomMotor.getConfigurator().apply(pidConfigs);
    }
}
