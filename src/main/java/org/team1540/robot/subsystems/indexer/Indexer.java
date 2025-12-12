package org.team1540.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (DriverStation.isDisabled()) io.stopAll();

        Logger.processInputs("Indexer", inputs);
    }

    public boolean getSensorTripped() {
        if (inputs.sensorConnected) {
            return inputs.sensorTripped;
        }
        return false;
    }

    public Command runIndexer(double percent) {
        return Commands.startEnd(
                () -> {
                    io.setRightVoltage(percent * 12.0);
                    io.setUpperVoltage(percent * 12.0);
                },
                io::stopAll);
    }

    public Command commandStopIndexer() {
        return Commands.runOnce(io::stopAll, this);
    }

    public static Indexer createReal() {
        return new Indexer(new IndexerIOReal());
    }

    public static Indexer createDummy() {
        return new Indexer(new IndexerIO() {});
    }
}
