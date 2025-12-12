package org.team1540.robot.subsystems.indexer;

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
        Logger.processInputs("Indexer", inputs);
    }

    public Command moveRightTo(double rotations) {
        return Commands.runEnd(() -> io.setRightMMPosition(rotations), io::stop, this);
    }

    public Command moveUpperTo(double rotations) {
        return Commands.runEnd(() -> io.setUpperMMPosition(rotations), io::stop, this);
    }

    public Command stopCommand() {
        return Commands.runOnce(io::stop, this);
    }

    public static Indexer createReal() {
        return new Indexer(new IndexerIOReal());
    }

    public static Indexer createDummy() {
        return new Indexer(new IndexerIO() {});
    }
}
