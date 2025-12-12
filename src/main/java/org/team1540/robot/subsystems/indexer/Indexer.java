package org.team1540.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

    public Indexer(IndexerIO indexerIO, FeederIO feederIO) {
        this.indexerIO = indexerIO;
        this.feederIO = feederIO;
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(indexerInputs);
        feederIO.updateInputs(feederInputs);

        if (DriverStation.isDisabled()) {
            indexerIO.stopAll();
            stopFeeder();
        }

        Logger.processInputs("Indexer", indexerInputs);
        Logger.processInputs("Feeder", feederInputs);
    }

    public boolean getSensorTripped() {
        if (indexerInputs.sensorConnected) {
            return indexerInputs.sensorTripped;
        }
        return false;
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

    public Command runIndexer(double percent) {
        return Commands.startEnd(
                () -> {
                    indexerIO.setRightVoltage(percent * 12.0);
                    indexerIO.setUpperVoltage(percent * 12.0);
                },
                indexerIO::stopAll);
    }

    public Command commandStopIndexer() {
        return Commands.runOnce(indexerIO::stopAll, this);
    }

    public Command commandRunFeeder(double percent) {
        return Commands.startEnd(() -> setFeederVolts(12 * percent), () -> setFeederVolts(0.0));
    }

    public static Indexer createReal() {
        return new Indexer(new IndexerIOReal(), new FeederIOReal());
    }

    public static Indexer createDummy() {
        return new Indexer(new IndexerIO() {}, new FeederIO() {});
    }
}
