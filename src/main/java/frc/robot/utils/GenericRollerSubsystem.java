package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.EndEffectorConstants;

public class GenericRollerSubsystem extends SubsystemBase {
    protected final TalonFX m_motor;
    protected final VoltageOut m_voltageOutReq = new VoltageOut(0);

    public GenericRollerSubsystem(int motorId, String canbus, boolean inverted) {
        m_motor = new TalonFX(motorId, canbus);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        m_motor.getConfigurator().apply(config);
    }

    public Command run(double voltage) {
        return runOnce(() -> {
            System.out.println("setting run to " + voltage);
            m_motor.setControl(m_voltageOutReq.withOutput(voltage));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            System.out.println("setting stop to 0");
            m_motor.setControl(m_voltageOutReq.withOutput(0));
        });
    }
}
