package frc.robot.subsystems.superstructure;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.*;
import frc.robot.subsystems.channel.Channel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffectorRollers;
import frc.robot.subsystems.endeffector.EndEffectorWrist;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;

public class Superstructure {
    private GPMode gpMode = GPMode.Coral;

    private final Elevator m_elevator;
    private final EndEffectorWrist m_eeWrist;
    private final EndEffectorRollers m_eeRollers;
    private final IntakeWrist m_intakeWrist;
    private final IntakeRollers m_intakeRollers;
    private final Channel m_channel;
    private final ElasticSender m_elastic;
    private long lastTime = 0;

    private AddressableLED m_led = new AddressableLED(0);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(22);
    private AddressableLEDBufferView m_leftLED = m_ledBuffer.createView(0, 6);
    private AddressableLEDBufferView m_rightLED = m_ledBuffer.createView(7, 14);
    private AddressableLEDBufferView m_topLED = m_ledBuffer.createView(15, 21);

    private LEDPattern hasCoralLED = LEDPattern.solid(Color.kGreen);
    private LEDPattern noCoralLED = LEDPattern.solid(Color.kBlue);

    private LEDPattern coralModeLED = LEDPattern.solid(Color.kWhite);
    private LEDPattern algaeModeLED = LEDPattern.solid(Color.kRed);

    public Superstructure(Elevator elevator, EndEffectorWrist eeWrist, EndEffectorRollers eeRollers,
            IntakeWrist intakeWrist, IntakeRollers intakeRollers, Channel channel, boolean debug) {
        this.m_elevator = elevator;
        this.m_eeWrist = eeWrist;
        this.m_eeRollers = eeRollers;
        this.m_intakeWrist = intakeWrist;
        this.m_intakeRollers = intakeRollers;
        this.m_channel = channel;

        m_elastic = new ElasticSender("Superstructure", debug);
        // m_elastic.addButton("Switch Mode", switchMode());
        // m_elastic.addButton("Intake", intake());
        // m_elastic.addButton("Score", score());
        // m_elastic.addButton("Eject Intake", ejectIntake());
        // m_elastic.addButton("Eject EE", ejectEE());
        // // m_elastic.addButton("Move L1", moveL1());
        // // m_elastic.addButton("Move L2", moveL2());
        // // m_elastic.addButton("Move L3", moveL3());
        // // m_elastic.addButton("Move L4", moveL4());
        // m_elastic.addButton("Stow", stow());

        hasCoralLED.applyTo(m_topLED);
        coralModeLED.applyTo(m_leftLED);
        coralModeLED.applyTo(m_rightLED);

        m_led.setData(m_ledBuffer);


    }

    public Command switchMode() {
        return Commands.runOnce(() -> {
            if (lastTime > System.currentTimeMillis() - 100) {
                return;
            }
            lastTime = System.currentTimeMillis();
            if (gpMode == GPMode.Coral) {
                gpMode = GPMode.Algae;
                System.out.println("switching mode to algae");

                algaeModeLED.applyTo(m_leftLED);
                algaeModeLED.applyTo(m_rightLED);

                m_led.setData(m_ledBuffer);

            } else {
                gpMode = GPMode.Coral;
                System.out.println("switching mode to coral");
                coralModeLED.applyTo(m_leftLED);
                coralModeLED.applyTo(m_rightLED);

                m_led.setData(m_ledBuffer);
            }
            m_elastic.put("gp mode", gpMode.toString(), false);
        });
    }

    public GPMode getGPMode() {
        if (gpMode == null)
            return GPMode.Coral;
        return gpMode;
    }
    public Command intakeCoral() {
        return Commands.sequence(
                Commands.runOnce ( () -> {
                    noCoralLED.applyTo(m_topLED);
                    m_led.setData(m_ledBuffer);
                }),
                m_elevator.moveTo(ElevatorConstants.INTAKE_HEIGHT),
                m_eeWrist.moveTo(EndEffectorWristPosition.INTAKE_CORAL_ANGLE),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.INTAKE_POSITION),
                m_intakeRollers.run(IntakeConstants.Rollers.INTAKE_CORAL_VOLTS),
                m_channel.run(ChannelConstants.CHANNEL_VOLTS),
                m_eeRollers.run(EndEffectorConstants.Rollers.INTAKE_CORAL_VOLTS),
                Commands.waitUntil(m_channel.coralInEndEffectorSupplier),
                Commands.runOnce ( () -> {
                    hasCoralLED.applyTo(m_topLED);
                    m_led.setData(m_ledBuffer);
                }),
                Commands.parallel(m_intakeRollers.stop(), m_channel.stop(),
                        m_eeRollers.run(EndEffectorConstants.Rollers.RETAIN_CORAL)),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION));
    }

    public Command intakeAlgae() {
        return Commands.sequence(
                Commands.runOnce ( () -> {
                    noCoralLED.applyTo(m_topLED);
                    m_led.setData(m_ledBuffer);
                }),
                m_eeWrist.moveTo(EndEffectorWristPosition.INTAKE_ALGAE_ANGLE),
                m_eeRollers.run(EndEffectorConstants.Rollers.INTAKE_ALGAE_VOLTS),
                Commands.waitUntil(() -> m_eeRollers.isStalled()),
                Commands.runOnce ( () -> {
                    hasCoralLED.applyTo(m_topLED);
                    m_led.setData(m_ledBuffer);
                }),
                m_eeRollers.stop(),
                m_eeRollers.run(EndEffectorConstants.Rollers.RETAIN_ALGAE));

        // m_intakeWrist.moveTo(IntakeConstants.Wrist.INTAKE_POSITION),
        // m_intakeRollers.run(IntakeConstants.Rollers.INTAKE_CORAL_VOLTS),
        // m_channel.run(ChannelConstants.CHANNEL_VOLTS),
        // m_eeRollers.run(EndEffectorConstants.Rollers.INTAKE_CORAL_VOLTS),
        // Commands.waitUntil(m_channel.coralInEndEffectorSupplier),
        // Commands.parallel(m_intakeRollers.stop(), m_channel.stop()),
        // m_intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION));
    }

    public Command intakeCoralWhileInAlgae() {
        return Commands.sequence(
                Commands.runOnce ( () -> {
                    noCoralLED.applyTo(m_topLED);
                    m_led.setData(m_ledBuffer);
                }),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.INTAKE_POSITION),
                m_intakeRollers.run(IntakeConstants.Rollers.INTAKE_CORAL_VOLTS),
                m_channel.run(ChannelConstants.CHANNEL_VOLTS),
                m_eeRollers.run(EndEffectorConstants.Rollers.INTAKE_CORAL_VOLTS),
                Commands.waitUntil(m_channel.coralInEndEffectorSupplier),
                Commands.runOnce ( () -> {
                    hasCoralLED.applyTo(m_topLED);
                    m_led.setData(m_ledBuffer);
                }),
                Commands.parallel(m_intakeRollers.stop(), m_channel.stop()),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION));
    }

    public Command intake() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, intakeCoral()),
                        Map.entry(GPMode.Algae, new PrintCommand("still in algae")/*intakeCoralWhileInAlgae()*/)),
                this::getGPMode);
    }

    public Command moveL1() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.L1_CORAL_HEIGHT),
                                m_eeWrist.moveTo(EndEffectorWristPosition.L1_SCORE_ANGLE))),
                        Map.entry(GPMode.Algae, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.PROCESSOR_ALGAE_HEIGHT),
                                m_intakeWrist.moveTo(IntakeConstants.Wrist.INTAKE_POSITION),
                                m_eeWrist.moveTo(EndEffectorWristPosition.SCORE_PROCESSOR_ANGLE)))),
                this::getGPMode);
    }

    public Command moveL2() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.L2_CORAL_HEIGHT),
                                m_eeWrist.moveTo(EndEffectorWristPosition.L2_PRE_ANGLE))),
                        Map.entry(GPMode.Algae, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.L2_ALGAE_HEIGHT),
                                intakeAlgae()))),
                this::getGPMode);
    }

    public Command moveL3() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.L3_CORAL_HEIGHT),
                                m_eeWrist.moveTo(EndEffectorWristPosition.L3_PRE_ANGLE))),
                        Map.entry(GPMode.Algae, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.L3_ALGAE_HEIGHT),
                                intakeAlgae()))),
                this::getGPMode);
    }

    public Command moveL4() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.L4_CORAL_HEIGHT),
                                m_eeWrist.moveTo(EndEffectorWristPosition.L4_PRE_ANGLE))),
                        Map.entry(GPMode.Algae, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.BARGE_ALGAE_HEIGHT),
                                m_eeWrist.moveTo(EndEffectorWristPosition.SCORE_BARGE_PRE_ANGLE)))),
                this::getGPMode);
    }

    public Command scoreCoral() {
        return Commands.runOnce(() -> {
            switch (m_eeWrist.getPosition()) {
                case L1_SCORE_ANGLE:
                    m_eeRollers.runFunc(EndEffectorConstants.Rollers.OUTTAKE_L1_CORAL_VOLTS);
                    break;
                case L2_PRE_ANGLE:
                case L3_PRE_ANGLE:
                    m_eeRollers.runFunc(EndEffectorConstants.Rollers.OUTTAKE_L2_L3_CORAL_VOLTS);
                    m_eeWrist.moveToNextPosition();
                    break;
                case L4_PRE_ANGLE:
                    m_eeRollers.runFunc(EndEffectorConstants.Rollers.OUTTAKE_L4_CORAL_VOLTS);
                    m_eeWrist.moveToNextPosition();
                    break;
                default:
                    m_eeRollers.runFunc(0);
                    break;
            }

            noCoralLED.applyTo(m_topLED);
            m_led.setData(m_ledBuffer);
        });
    }

    public Command scoreAlgae() {
        return Commands.runOnce(() -> {
            switch (m_eeWrist.getPosition()) {
                case SCORE_PROCESSOR_ANGLE:
                    m_eeRollers.runFunc(EndEffectorConstants.Rollers.OUTTAKE_PROCCESOR_VOLTS);
                    break;
                case SCORE_BARGE_PRE_ANGLE:
                    m_eeWrist.moveToFunc(EndEffectorWristPosition.SCORE_BARGE_ANGLE, EndEffectorWristSide.BACK);
                    m_eeRollers.runFunc(EndEffectorConstants.Rollers.OUTTAKE_BARGE_VOLTS);
                    break;
                default:
                    return;
            }

            noCoralLED.applyTo(m_topLED);
            m_led.setData(m_ledBuffer);
        });
    }

    public Command ejectIntake() {
        return Commands.sequence(
                m_intakeWrist.moveTo(IntakeConstants.Wrist.EJECT_POSITION),
                m_intakeRollers.run(IntakeConstants.Rollers.EJECT_VOLTS),
                m_channel.run(ChannelConstants.EJECT_VOLTS));
    }

    public Command ejectEE() {
        return Commands.sequence(
                m_eeWrist.moveTo(EndEffectorWristPosition.L2_PRE_ANGLE),
                m_eeRollers.run(EndEffectorConstants.Rollers.EJECT_VOLTS),
                m_elevator.moveTo(ElevatorConstants.STOWED_HEIGHT));
    }

    public Command score() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, scoreCoral()),
                        Map.entry(GPMode.Algae, scoreAlgae())),
                this::getGPMode);
    }

    public Command stow() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.STOWED_HEIGHT),
                m_eeWrist.moveTo(EndEffectorWristPosition.STOW_ANGLE),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION),
                m_intakeRollers.stop(),
                m_eeRollers.stop(),
                m_channel.stop());
    }
}
