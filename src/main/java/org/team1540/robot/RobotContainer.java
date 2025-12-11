package org.team1540.robot;

import static org.team1540.robot.Constants.Mode.REAL;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot.subsystems.drive.Drivetrain;
import org.team1540.robot.subsystems.vision.apriltag.AprilTagVision;

public class RobotContainer {

    Drivetrain drivetrain;
    AprilTagVision aprilTagVision;
    CommandXboxController driver = new CommandXboxController(0);
    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                drivetrain = Drivetrain.createReal();
                aprilTagVision = AprilTagVision.createReal();
        }
        configureButtonBindings();
        configureAutoRoutines();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID(), () -> true));
    }

    private void configureAutoRoutines() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
