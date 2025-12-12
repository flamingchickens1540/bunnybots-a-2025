package org.team1540.robot.subsystems.shooter;

import static org.team1540.robot.subsystems.shooter.ShooterConstants.Pivot.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private double leftFlywheelSetpointRPM;
    private double rightFlywheelSetpointRPM;

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

    private final LinearFilter leftSpeedFilter = LinearFilter.movingAverage(20);
    // Units: RPM
    private final LinearFilter rightSpeedFilter = LinearFilter.movingAverage(20);
    private final LinearFilter pivotPositionFilter = LinearFilter.movingAverage(10);

    // todo shooter lerp
    private static boolean hasInstance = false;

    /*
     * Function: Constructor - DONE
     * Function: Periodic - DONE
     * Function: Create real [Leave until end]
     *
     * for feeder:
     * Function: set speed - DONE
     * Function: set volts - DONE
     * Function: stop flywheels - DONE
     * Function: Get left/right speed - DONE
     * Function: get Spun-up % - DONE
     * Function: get Spun-up boolean - DONE
     *
     * for pivot:
     * Function: set pivot pos (pos2D)
     * Function: holds pivot position
     * Function: set poivot volts
     * Function: stop pivot
     * Function: is pivot at set point?
     *
     * for feeder:
     * Function: set speed - DONE
     * Function: set volts - DONE
     * Function: stop feeder - DONE
     * Fuunction: get speed - DONE
     *
     * Spin up command - DONE
     * Set pivot pos command
     * Spin up and set pivot pos command
     *
     * FEEDER COMMANDS [TBD]
     */

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
        leftSpeedFilter.calculate(getLeftFlywheelSpeed());
        rightSpeedFilter.calculate(getRightFlywheelSpeed());
        pivotPositionFilter.calculate(getPivotPosition().getRotations());
        Logger.recordOutput("Shooter/Pivot/Error", pivotSetpoint.getDegrees() - pivotInputs.position.getDegrees());
    }

    // Fly wheel functions
    public void setFlywheelSpeeds(double leftSpeedRPM, double rightSpeedRPM) {
        leftFlywheelSetpointRPM = leftSpeedRPM;
        rightFlywheelSetpointRPM = rightSpeedRPM;
        flywheelsIO.setSpeeds(leftSpeedRPM, rightSpeedRPM);
    }

    public void setFlywheelVolts(double rightVolts, double leftVolts) {
        flywheelsIO.setVoltage(MathUtil.clamp(rightVolts, -12, 12), MathUtil.clamp(leftVolts, -12, 12));
    }

    public double getRightFlywheelSpeed() {
        return flywheelInputs.rightVelocityRPM;
    }

    public double getLeftFlywheelSpeed() {
        return flywheelInputs.leftVelocityRPM;
    }

    public double getSpinUpPercent() {
        return (getRightFlywheelSpeed() + getLeftFlywheelSpeed())
                / (getRightFlywheelSetpointRPM() + getLeftFlywheelSetpointRPM());
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
    // todo check to see if set pivot brake mode is needed

    // Feeder functions
    public void setFeederSpeed(double speedRPM) {
        feederSetpointRPM = speedRPM;
        feederIO.setVoltage(speedRPM);
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

        if (getSpinUpPercent() >= 0.95) {
            return true;
        } else {
            return false;
        }
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
                () -> {
                    setPivotPosition(setpoint.get());
                },
                this);
    }

    public Command spinUpCommand(Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint) {
        return Commands.run(
                () -> {
                    setFlywheelSpeeds(leftSetpoint.get(), rightSetpoint.get());
                    // code to exacut
                },
                this);
    }

    public Command spinUpAndSetPivotPosition(
            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Rotation2d> setpoint) {
        return Commands.run(() -> {
            setFlywheelSpeeds(leftSetpoint.get(), rightSetpoint.get());
            setPivotPosition(setpoint.get());
        });
    }

    @AutoLogOutput(key = "Shooter/Pivot/PivotSetpoint")
    public Rotation2d getPivotSetpoint() {
        return pivotSetpoint;
    }

    public Command commandZeroPivot() {
        return Commands.runOnce(() -> setPivotVolts(0.3 * 12))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(Commands.waitUntil(() -> Math.abs(pivotInputs.currentStatorAmps) > HARD_STOP_CURRENT)
                        .andThen(Commands.runOnce(() -> setPivotPosition(getPivotPosition()))))
                .andThen(this::resetEncodePosition);
    }

    public void resetEncodePosition() {
        pivotIO.setEncoderPosition(Rotation2d.kZero);
    }

    @AutoLogOutput(key = "Shooter/Flywheels/leftSetpointRPM")
    public double getLeftFlywheelSetpointRPM() {
        return leftFlywheelSetpointRPM;
    }

    @AutoLogOutput(key = "Shooter/Flywheels/rightSetpointRPM")
    public double getRightFlywheelSetpointRPM() {
        return rightFlywheelSetpointRPM;
    }

    @AutoLogOutput(key = "Shooter/Feeder/feederSetpointRPM")
    public double getFeederSetpointRPM() {
        return feederSetpointRPM;
    }
}
