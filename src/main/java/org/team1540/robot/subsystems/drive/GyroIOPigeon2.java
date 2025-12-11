package org.team1540.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import org.team1540.robot.generated.TunerConstants;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon =
            new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.DrivetrainConstants.CANBusName);
    ;

    private final StatusSignal<Angle> yawPosition = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    private final Queue<Double> timestampQueue;
    private final Queue<Double> yawPositionQueue;

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(yawPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50, yawVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(DrivetrainConstants.ODOMETRY_FREQUENCY_HZ, yawPosition);
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(yawVelocity, yawPosition);
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        yawPositionQueue.clear();
        timestampQueue.clear();

        inputs.isConnected = pigeon.isConnected();
        inputs.yawPosition = Rotation2d.fromRotations(yawPosition.getValueAsDouble());
        inputs.yawVelocityRadsPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }
}
