package org.team1540.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team1540.robot.commands.CharacterizationCommands;
import org.team1540.robot.subsystems.drive.Drivetrain;
import org.team1540.robot.subsystems.intake.Intake;
import org.team1540.robot.subsystems.shooter.Shooter;
import org.team1540.robot.subsystems.vision.apriltag.AprilTagVision;

public class RobotContainer {
    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    private final Drivetrain drivetrain;
    private final AprilTagVision vision;
    private final Shooter shooter;
    private final Intake intake;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                drivetrain = Drivetrain.createReal();
                vision = AprilTagVision.createReal();
                shooter = Shooter.createReal();
                intake = Intake.createReal();
            }
            default -> {
                drivetrain = Drivetrain.createDummy();
                vision = AprilTagVision.createDummy();
                shooter = Shooter.createDummy();
                intake = Intake.createDummy();
            }
        }

        configureButtonBindings();
        configureAutoRoutines();
    }

    private void configureButtonBindings() {
        // Tuning commands
        copilot.start().and(copilot.back()).toggleOnTrue(shooter.tuningCommand().unless(() -> !Constants.isTuningMode()));

        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID(), () -> true));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));
    }

    private void configureAutoRoutines() {
        if (Constants.isTuningMode()) {
            autoChooser.addOption("Shooter FF Char", CharacterizationCommands.feedforward(
                    volts -> shooter.setFlywheelVolts(volts, volts),
                    () -> ((shooter.getBottomFlywheelSpeed() + shooter.getTopFlywheelSpeed()) / 2.0) / 60.0,
                    shooter));
        }
        autoChooser.addOption("Zero Mechanisms", Commands.parallel(shooter.commandZeroPivot(), intake.commandZeroPivot()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
