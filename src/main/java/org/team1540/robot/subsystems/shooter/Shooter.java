package org.team1540.robot.subsystems.shooter;

import static org.team1540.robot.subsystems.shooter.ShooterConstants.Pivot.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.Constants;
import org.team1540.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {

    private final ShooterPivotIO pivotIO;
    private final ShooterPivotIOInputsAutoLogged pivotInputs = new ShooterPivotIOInputsAutoLogged();

    private final FlywheelsIO flywheelsIO;
    private final FlywheelsIOInputsAutoLogged flywheelInputs = new FlywheelsIOInputsAutoLogged();

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

    // Units: rotations

    private double topFlywheelSetpointRPM;
    private double bottomFlywheelSetpointRPM;

    private Rotation2d pivotSetpoint = new Rotation2d();

    private double feederSetpointRPM;

    private final LoggedTunableNumber pivotKP = new LoggedTunableNumber("Shooter/Pivot/kP", ShooterConstants.Pivot.KP);
    private final LoggedTunableNumber pivotKI = new LoggedTunableNumber("Shooter/Pivot/kI", ShooterConstants.Pivot.KI);
    private final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Shooter/Pivot/kD", ShooterConstants.Pivot.KD);
    private final LoggedTunableNumber pivotKG = new LoggedTunableNumber("Shooter/Pivot/kG", ShooterConstants.Pivot.KG);

    private final LoggedTunableNumber flywheelsKP =
            new LoggedTunableNumber("Shooter/Flywheels/kP", ShooterConstants.Flywheels.KP);
    private final LoggedTunableNumber flywheelsKI =
            new LoggedTunableNumber("Shooter/Flywheels/kI", ShooterConstants.Flywheels.KI);
    private final LoggedTunableNumber flywheelsKD =
            new LoggedTunableNumber("Shooter/Flywheels/kD", ShooterConstants.Flywheels.KD);
    private final LoggedTunableNumber flywheelsKV =
            new LoggedTunableNumber("Shooter/Flywheels/kV", ShooterConstants.Flywheels.KV);
    private final LoggedTunableNumber flywheelsKS =
            new LoggedTunableNumber("Shooter/Flywheels/kS", ShooterConstants.Flywheels.KS);

    private final LinearFilter topSpeedFilter = LinearFilter.movingAverage(20);
    // Units: RPM
    private final LinearFilter bottomSpeedFilter = LinearFilter.movingAverage(20);
    private final LinearFilter pivotPositionFilter = LinearFilter.movingAverage(10);

    // todo shooter lerp
    private static boolean hasInstance = false;

    private Shooter(ShooterPivotIO pivotIO, FlywheelsIO flywheelsIO, FeederIO feederIO) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        hasInstance = true;
        this.pivotIO = pivotIO;
        this.flywheelsIO = flywheelsIO;
        this.feederIO = feederIO;
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotInputs);
        flywheelsIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);

        if (RobotState.isDisabled()) {
            stopFlywheels();
            stopPivot();
        }
        // Update tunable numbers
        if (Constants.isTuningMode()
                && (flywheelsKP.hasChanged(hashCode())
                        || flywheelsKI.hasChanged(hashCode())
                        || flywheelsKD.hasChanged(hashCode())
                        || flywheelsKV.hasChanged(hashCode()))) {
            flywheelsIO.configPID(
                    flywheelsKP.get(), flywheelsKI.get(), flywheelsKD.get(), flywheelsKV.get(), flywheelsKS.get());
        }
        if (Constants.isTuningMode()
                && (pivotKP.hasChanged(hashCode())
                        || pivotKI.hasChanged(hashCode())
                        || pivotKD.hasChanged(hashCode())
                        || pivotKG.hasChanged(hashCode()))) {
            pivotIO.configPID(pivotKP.get(), pivotKI.get(), pivotKD.get(), pivotKG.get());
        }

        // Add values to filters
        topSpeedFilter.calculate(getTopFlywheelSpeed());
        bottomSpeedFilter.calculate(getBottomFlywheelSpeed());
        pivotPositionFilter.calculate(getPivotPosition().getRotations());
        Logger.recordOutput("Shooter/Pivot/Error", pivotSetpoint.getDegrees() - pivotInputs.position.getDegrees());
    }

    // Fly wheel functions
    public void setFlywheelSpeeds(double topSpeedRPM, double bottomSpeedRPM) {
        topFlywheelSetpointRPM = topSpeedRPM;
        bottomFlywheelSetpointRPM = bottomSpeedRPM;
        flywheelsIO.setSpeeds(topSpeedRPM, bottomSpeedRPM);
    }

    public void setFlywheelVolts(double rightVolts, double leftVolts) {
        flywheelsIO.setVoltage(MathUtil.clamp(rightVolts, -12, 12), MathUtil.clamp(leftVolts, -12, 12));
    }

    public double getBottomFlywheelSpeed() {
        return flywheelInputs.bottomVelocityRPM;
    }

    public double getTopFlywheelSpeed() {
        return flywheelInputs.topVelocityRPM;
    }

    public double getSpinUpPercent() {
        return (getBottomFlywheelSpeed() + getTopFlywheelSpeed())
                / (getBottomFlywheelSetpointRPM() + getTopFlywheelSetpointRPM());
    }

    public void stopFlywheels() {
        setFlywheelVolts(0, 0);
    }

    // Pivot functions

    public void setPivotPosition(Rotation2d position) {
        pivotSetpoint = Rotation2d.fromRotations(MathUtil.clamp(
                position.getRotations(),
                ShooterConstants.Pivot.MIN_ANGLE_ROTS.getRotations(),
                ShooterConstants.Pivot.MAX_ANGLE_ROTS.getRotations()));
        pivotPositionFilter.reset();
        pivotIO.setPosition(pivotSetpoint);
    }

    public void holdPivotPosition() {
        //        pivotPositionFilter.clear();
        pivotIO.setPosition(pivotSetpoint);
    }

    public void setPivotVolts(double volts) {
        pivotIO.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    /**
     * Applies neutral output to the pivot
     */
    public void stopPivot() {
        setPivotVolts(0);
    }

    public void setFeederVolts(double volts) {
        feederIO.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    public void stopFeeder() {
        setFeederVolts(0);
    }

    public double getFeederSpeed() {
        return feederInputs.feederVelocityRPM;
    }

    /*  Command factories
    public Command spinUpCommand() {}
    public Command setPivotPositionCommand() {}
    public Command spinUpAndSetPivotPosition() {}
    */

    public boolean areFlywheelsSpunUp() {
        return getSpinUpPercent() >= 0.95;
    }

    public Rotation2d getPivotPosition() {
        return pivotInputs.position;
    }

    public boolean isPivotAtSetpoint() {
        return MathUtil.isNear(
                pivotSetpoint.getRotations(),
                pivotPositionFilter.calculate(getPivotPosition().getRotations()),
                ShooterConstants.Pivot.ERROR_TOLERANCE_ROTS);
    }

    public Command setPivotPositionCommand(Supplier<Rotation2d> setpoint) {
        return Commands.run(
                () -> setPivotPosition(setpoint.get()),
                this);
    }

    public Command spinUpCommand(Supplier<Double> topSetpoint, Supplier<Double> bottomSetpoint) {
        return Commands.run(
                () -> setFlywheelSpeeds(topSetpoint.get(), bottomSetpoint.get()),
                this);
    }

    public Command spinUpAndSetPivotPosition(
            DoubleSupplier topSetpoint, DoubleSupplier bottomSetpoint, Supplier<Rotation2d> setpoint) {
        return Commands.runEnd(() -> {
            setFlywheelSpeeds(topSetpoint.getAsDouble(), bottomSetpoint.getAsDouble());
            setPivotPosition(setpoint.get());
        }, this::stopFlywheels);
    }

    @AutoLogOutput(key = "Shooter/Pivot/PivotSetpoint")
    public Rotation2d getPivotSetpoint() {
        return pivotSetpoint;
    }

    public Command commandZeroPivot() {
        return Commands.runOnce(() -> setPivotVolts(0.3 * 12))
                .andThen(
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(() -> Math.abs(pivotInputs.currentStatorAmps) > HARD_STOP_CURRENT),
                        Commands.runOnce(() -> resetEncoderPosition(Rotation2d.kCCW_90deg)),
                        Commands.runOnce(() -> setPivotPosition(Rotation2d.kCCW_90deg)));
    }

    public Command tuningCommand() {
        LoggedTunableNumber topSpeedRPM = new LoggedTunableNumber("Shooter/Tuning/TopSpeedRPM", 5000);
        LoggedTunableNumber bottomSpeedRPM = new LoggedTunableNumber("Shooter/Tuning/BottomSpeedRPM", 5000);
        LoggedTunableNumber pivotAngleDegrees = new LoggedTunableNumber("Shooter/Tuning/PivotAngleDegrees", 45);
        return spinUpAndSetPivotPosition(
                topSpeedRPM, bottomSpeedRPM, () -> Rotation2d.fromDegrees(pivotAngleDegrees.getAsDouble()));
    }

    public void resetEncoderPosition(Rotation2d position) {
        pivotIO.setEncoderPosition(position);
    }

    @AutoLogOutput(key = "Shooter/Flywheels/topSetpointRPM")
    public double getTopFlywheelSetpointRPM() {
        return topFlywheelSetpointRPM;
    }

    @AutoLogOutput(key = "Shooter/Flywheels/bottomSetpointRPM")
    public double getBottomFlywheelSetpointRPM() {
        return bottomFlywheelSetpointRPM;
    }

    @AutoLogOutput(key = "Shooter/Feeder/feederSetpointRPM")
    public double getFeederSetpointRPM() {
        return feederSetpointRPM;
    }

    public static Shooter createReal() {
        return new Shooter(new ShooterPivotIOReal(), new FlywheelsIOReal(), new FeederIOReal());
    }

    public static Shooter createDummy() {
        return new Shooter(new ShooterPivotIO() {}, new FlywheelsIO() {}, new FeederIO() {});
    }
}
