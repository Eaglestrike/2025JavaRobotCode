package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.utils.GenericRollerSubsystem;

public class EndEffectorRollers extends GenericRollerSubsystem {
    private EagleSwerveDrivetrain drivetrain;

    public EndEffectorRollers(EagleSwerveDrivetrain drivetrain) {
        super(EndEffectorConstants.Rollers.MOTOR_ID, "rio", false);
        this.drivetrain = drivetrain;
    }

    public void runFunc(double voltage) {
        System.out.println("setting voltage to " + voltage);
        m_motor.setControl(new VoltageOut(voltage));
    }

    public boolean isStalled() {
        return Math.abs(m_motor.getStatorCurrent().getValueAsDouble()) > EndEffectorConstants.Rollers.STALL_CURRENT;
    }

    public Command holdAlgae() {
        return Commands.runOnce(() -> run(EndEffectorConstants.Rollers.RETAIN_ALGAE));
    }

    public Command stopHoldingAlgae() {
        return Commands.runOnce(() -> stop());
    }

    @Override
    public void periodic() {
    }

    public Command setManualVoltage(double joystickPosition) {
        return run(
                () -> {
                    m_motor.setVoltage(joystickPosition * EndEffectorConstants.Rollers.MAX_VOLTS
                            / EndEffectorConstants.Rollers.MANUAL_RATIO);
                });
    }

    @Override
    public Command stop() {
        return runOnce(() -> {
            if (!EndEffectorSideUtils.facingReef(drivetrain.getState().Pose)) {
                System.out.println("retain on ee rollers");
                m_motor.setControl(new VoltageOut(EndEffectorConstants.Rollers.RETAIN_CORAL));
            } else {
                m_motor.setControl(new VoltageOut(0));
            }
        });
    }

    public Command fullStop() {
        return runOnce(() -> {
            System.out.println("full stop");
            m_motor.setControl(new VoltageOut(0));
        });
    }
}
