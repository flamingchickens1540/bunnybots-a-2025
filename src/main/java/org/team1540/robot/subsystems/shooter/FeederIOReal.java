package org.team1540.robot.subsystems.shooter;

import static org.team1540.robot.subsystems.shooter.ShooterConstants.Feeder.*;
import static org.team1540.robot.subsystems.shooter.ShooterConstants.Flywheels.KD;
import static org.team1540.robot.subsystems.shooter.ShooterConstants.Flywheels.KI;
import static org.team1540.robot.subsystems.shooter.ShooterConstants.Flywheels.KP;
import static org.team1540.robot.subsystems.shooter.ShooterConstants.Flywheels.KS;
import static org.team1540.robot.subsystems.shooter.ShooterConstants.Flywheels.KV;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOReal implements FeederIO {

    private final TalonFX feederMotor = new TalonFX(FEEDER_ID);

    private final StatusSignal<AngularVelocity> feederVelocity = feederMotor.getVelocity();
    private final StatusSignal<Voltage> feederAppliedVolts = feederMotor.getMotorVoltage();
    private final StatusSignal<Current> feederCurrent = feederMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> feederTempCelsius = feederMotor.getDeviceTemp();

    private final VelocityVoltage feederVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut feederVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    public FeederIOReal() {
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
        feederMotor.getConfigurator().apply(config);

        // Sets update freqency for the listed values while disabling any other status signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                50, feederVelocity, feederAppliedVolts, feederCurrent, feederTempCelsius);

        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(feederVelocity, feederAppliedVolts, feederCurrent, feederTempCelsius);

        inputs.feederVelocityRPM = feederVelocity.getValueAsDouble() * 60;
        inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
        inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
        inputs.feederTempCelsius = feederTempCelsius.getValueAsDouble();
    }

    @Override
    public void setSpeed(double RPM) {
        feederMotor.setControl(feederVelocityCtrlReq.withVelocity(RPM / 60));
    }

    @Override
    public void setVoltage(double volts) {
        feederMotor.setControl(feederVoltageCtrlReq.withOutput(volts));
    }
}
