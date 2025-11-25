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
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.Constants;

public class Intake extends SubsystemBase {
    public static boolean hasInstance = false;

    public enum IntakeState {
        //add the states

        private final DoubleSupplier pivotPosition;

        IntakeState(DoubleSupplier pivotPositionDeg) {
            this.pivotPosition = pivotPositionDeg;
        }

        public Rotation2d pivotPosition() {
            return Rotation2d.fromDegrees(pivotPosition.getAsDouble());
        }
    }

    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private Intake(IntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {

    }
}
