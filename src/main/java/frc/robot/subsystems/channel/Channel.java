package frc.robot.subsystems.channel;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.ChannelConstants;
import frc.robot.constants.IntakeConstants;

public class Channel extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ChannelConstants.CHANNEL_MOTOR_ID, "rio");
    private LaserCan distanceSensor;
    private final Queue<Integer> distanceMeasurements = new ArrayDeque<>();

    private boolean coralInEndEffector = false;

    public BooleanSupplier coralInEndEffectorSupplier = () -> coralInEndEffector;
    private ElasticSender m_sender;

    public Channel(boolean debug) {
        m_sender = new ElasticSender("Channel", debug);
        // m_sender.addButton("Stop", stop());
        // m_sender.addButton("Run", run(ChannelConstants.CHANNEL_VOLTS));
        m_sender.put("in channel", coralInEndEffector, false);


        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs motorConfigs = cfg.MotorOutput;
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Coast;

        cfg.Voltage.PeakForwardVoltage = ChannelConstants.CHANNEL_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -ChannelConstants.CHANNEL_VOLTS;
        cfg.CurrentLimits.StatorCurrentLimit = ChannelConstants.STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = ChannelConstants.SUPPLY_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_motor.getConfigurator().apply(cfg);

        distanceSensor = new LaserCan(ChannelConstants.DISTANCE_SENSOR_ID);
    }

    public Command run(double voltage) {
        return runOnce(() -> m_motor.setVoltage(voltage));
    }

    public Command stop() {
        return runOnce(() -> m_motor.setVoltage(0));
    }

    public Command setManualVoltage(double joystickPosition) {
        return run(
                () -> {
                    m_motor.setVoltage(joystickPosition * ChannelConstants.CHANNEL_VOLTS
                            / ChannelConstants.MANUAL_RATIO);
                });
    }

    @Override
    public void periodic() {
        m_sender.put("in channel", coralInEndEffector, false);
        LaserCan.Measurement measurement = distanceSensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            distanceMeasurements.add(measurement.distance_mm);

            if (distanceMeasurements.size() > ChannelConstants.MEASUREMENT_BUFFER_SIZE) {
                distanceMeasurements.poll(); // Removes the oldest measurement
            }

            if (distanceMeasurements.size() == ChannelConstants.MEASUREMENT_BUFFER_SIZE) {
                boolean allBelowThreshold = true;
                for (double dist : distanceMeasurements) {
                    if (dist >= ChannelConstants.DISTANCE_SENSOR_THRESH) {
                        allBelowThreshold = false;
                        break;
                    }
                }
                coralInEndEffector = allBelowThreshold;
            } else {
                coralInEndEffector = false;
            }
        } else {
            coralInEndEffector = false;
            distanceMeasurements.clear();
        }
        if (RobotBase.isSimulation()) {
            coralInEndEffector = true;
        }
    }
}
