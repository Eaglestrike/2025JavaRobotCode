package frc.robot.subsystems.channel;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.ChannelConstants;


public class Channel extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ChannelConstants.CHANNEL_MOTOR_ID, "rio");
    private LaserCan distanceSensor;

    private boolean coralInEndEffector = false;

    public BooleanSupplier coralInEndEffectorSupplier = () -> coralInEndEffector;
    private ElasticSender m_sender;

    public Channel(boolean debug) {
        m_sender = new ElasticSender("Channel", debug);
        m_sender.addButton("Stop", stop());
        m_sender.addButton("Run", run(ChannelConstants.CHANNEL_VOLTS));

        distanceSensor = new LaserCan(ChannelConstants.DISTANCE_SENSOR_ID);
    }

    public Command run(double voltage) {
        return runOnce(() -> m_motor.setVoltage(voltage));
    }

    public Command stop() {
        return runOnce(() -> m_motor.setVoltage(0));
    }

    @Override
    public void periodic() {
        LaserCan.Measurement measurement = distanceSensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            coralInEndEffector = measurement.distance_mm < ChannelConstants.DISTANCE_SENSOR_THRESH;
        } else {
            coralInEndEffector = false;
        }
    }
}
