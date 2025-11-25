package org.team1540.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterPivotIO pivotIO;
    private final ShooterPivotIOInputsAutoLogged pivotInputs = new ShooterPivotIOInputsAutoLogged();

    private final FlywheelsIO flywheelsIO;
    private final FlywheelsIOInputsAutoLogged flywheelInputs = new FlywheelsIOInputsAutoLogged();

    private double leftFlywheelSetpointRPM;
    private double rightFlywheelSetpointRPM;

    private static boolean hasInstance = false;

    /*
     * Function: Constructor - DONE
     * Function: Periodic - DONE
     * Function: Create real [Leave until end]
     *
     *
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
     * Spin up command - DONE
     * Set pivot pos command
     * Spin up and set pivot pos command
     */

    private Shooter(ShooterPivotIO pivotIO, FlywheelsIO flywheelsIO) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        hasInstance = true;
        this.pivotIO = pivotIO;
        this.flywheelsIO = flywheelsIO;
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

    public void stopPivot() {}

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

    public Command spinUpCommand(Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint) {
        return new FunctionalCommand(
                () -> {},
                () -> setFlywheelSpeeds(leftSetpoint.get(), rightSetpoint.get()),
                (ignored) -> {},
                this::areFlywheelsSpunUp,
                this);
    }

    @AutoLogOutput(key = "Shooter/Flywheels/leftSetpointRPM")
    public double getLeftFlywheelSetpointRPM() {
        return leftFlywheelSetpointRPM;
    }

    @AutoLogOutput(key = "Shooter/Flywheels/rightSetpointRPM")
    public double getRightFlywheelSetpointRPM() {
        return rightFlywheelSetpointRPM;
    }
}
