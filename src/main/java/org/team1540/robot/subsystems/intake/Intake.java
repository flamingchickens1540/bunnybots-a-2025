package org.team1540.robot.subsystems.intake;

import static org.team1540.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot.Constants;
import org.team1540.robot.util.LoggedTunableNumber;

// All code that was causing errors due to the lack of utils is commented out

public class Intake extends SubsystemBase {
    public static boolean hasInstance = false;

    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private final LoggedTunableNumber pivotKP = new LoggedTunableNumber("Intake/kP", PIVOT_KP);
    private final LoggedTunableNumber pivotKI = new LoggedTunableNumber("Intake/kI", PIVOT_KI);
    private final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Intake/kD", PIVOT_KD);
    private final LoggedTunableNumber pivotKS = new LoggedTunableNumber("Intake/kS", PIVOT_KS);
    private final LoggedTunableNumber pivotKV = new LoggedTunableNumber("Intake/kV", PIVOT_KV);
    private final LoggedTunableNumber pivotKG = new LoggedTunableNumber("Intake/kG", PIVOT_KG);

    private final Alert pivotDisconnectedAlert = new Alert("Intake pivot disconnected", Alert.AlertType.kError);
    private final Alert rollerDisconnectedAlert = new Alert("Intake roller disconnected", Alert.AlertType.kError);

    private Rotation2d pivotSetpoint = PIVOT_MIN_ANGLE;

    private final TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(PIVOT_CRUISE_VELOCITY_RPS, PIVOT_ACCELERATION_RPS2));

    private Intake(IntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            stopAll();
        }

        // LoggedTracer.reset();
        pivotDisconnectedAlert.set(!inputs.pivotConnected);
        rollerDisconnectedAlert.set(!inputs.rollerConnected);

        io.updateInputs(inputs);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setPivotPID(pivotKP.get(), pivotKI.get(), pivotKD.get()),
                pivotKP,
                pivotKI,
                pivotKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setPivotFF(pivotKS.get(), pivotKV.get(), pivotKG.get()),
                pivotKS,
                pivotKV,
                pivotKG);

        if (DriverStation.isDisabled()) stopAll();

        pivotDisconnectedAlert.set(!inputs.pivotConnected);
        rollerDisconnectedAlert.set(!inputs.rollerConnected);

        // LoggedTracer.record("Intake");
    }

    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public void setPivotPosition(Rotation2d rotations) {
        pivotSetpoint = rotations;
        io.setPivotSetpoint(rotations);
    }

    public Rotation2d getPivotPosition() {
        return inputs.pivotPosition;
    }

    public void setPivotVoltage(double voltage) {
        io.setPivotVoltage(voltage);
    }

    public void resetPivotPosition(Rotation2d rotations) {
        io.resetPivotPosition(rotations);
    }

    public void stopAll() {
        setRollerVoltage(0);
        setPivotVoltage(0);
    }

    public void holdPivot() {
        setPivotPosition(inputs.pivotPosition);
    }

    @AutoLogOutput(key = "Intake/PivotAtSetpoint")
    public boolean isPivotAtSetpoint() {
        return MathUtil.isNear(pivotSetpoint.getDegrees(), inputs.pivotPosition.getDegrees(), 3.0);
    }

    @AutoLogOutput(key = "Intake/PivotSetpoint")
    public Rotation2d getPivotSetpoint() {
        return pivotSetpoint;
    }

    public Command commandRunRoller(double percent) {
        return Commands.startEnd(() -> this.setRollerVoltage(percent * 12), () -> this.setRollerVoltage(0), this);
    }

    public Command commandZeroPivot() {
        return Commands.runOnce(() -> setPivotVoltage(0.3 * 12))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(Commands.waitUntil(() -> Math.abs(inputs.pivotStatorCurrentAmps) > 20)
                        .andThen(Commands.runOnce(() -> resetPivotPosition(Rotation2d.fromDegrees(90))))
                        .andThen(commandToStow()));
    }

    public double timeToSetpoint(Rotation2d setpoint) {
        trapezoidProfile.calculate(
                0.0,
                new TrapezoidProfile.State(getPivotPosition().getRotations(), inputs.pivotMotorVelocityRPS),
                new TrapezoidProfile.State(setpoint.getRotations(), 0));
        return trapezoidProfile.totalTime();
    }

    public Command commandToStow() {
        return (Commands.run(() -> setPivotPosition(PIVOT_STOW_ANGLE)).until(this::isPivotAtSetpoint))
                .handleInterrupt(this::holdPivot);
    }

    public Command commandToIntake() {
        return (Commands.run(() -> setPivotPosition(PIVOT_INTAKE_ANGLE)).until(this::isPivotAtSetpoint))
                .andThen(commandRunRoller(67.0))
                .handleInterrupt(this::holdPivot);
    }

    public static Intake createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real intake on simulated robot", false);
        }
        return new Intake(new IntakeIOReal());
    }

    public static Intake createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy intake on real robot", false);
        }
        return new Intake(new IntakeIO() {});
    }
}
