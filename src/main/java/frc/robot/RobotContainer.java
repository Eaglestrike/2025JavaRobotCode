// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.DriveUntilStall;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.constants.AutoAlignPosition;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.Drive.EagleSwerveTelemetry;
import frc.robot.subsystems.channel.Channel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffectorRollers;
import frc.robot.subsystems.endeffector.EndEffectorWrist;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.superstructure.GPMode;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.vision.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // swerve
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT)
            .withRotationalDeadband(MaxAngularRate * DriveConstants.DRIVE_DEADBAND_MULT)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.ApplyRobotSpeeds strafeDrive = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final EagleSwerveTelemetry logger = new EagleSwerveTelemetry(MaxSpeed);
    public final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision(drivetrain);

    private boolean slowModeOn = false;
    private AutoAlignPosition autoAlignPosition = AutoAlignPosition.A;
    private Supplier<Pose2d> autoAlignPositionSupplier = () -> AutoAlignConstants.REEF_POSITIONS.get(autoAlignPosition);

    // controllers
    private final CommandJoystick m_leftJoystick = new CommandJoystick(0);
    private final CommandJoystick m_rightJoystick = new CommandJoystick(1);
    private final CommandXboxController m_operatorController = new CommandXboxController(2);
    private final CommandXboxController m_buttonBoard = new CommandXboxController(3);

    // auto
    private final SendableChooser<Command> autoChooser;

    private final Elevator m_elevator = new Elevator(true);
    private final EndEffectorWrist m_eeWrist = new EndEffectorWrist(true, drivetrain, vision);
    private final EndEffectorRollers m_eeRollers = new EndEffectorRollers();
    private final IntakeWrist m_intakeWrist = new IntakeWrist(true);
    private final IntakeRollers m_intakeRollers = new IntakeRollers();
    private final Channel m_channel = new Channel(true);
    private final Superstructure m_superstructure = new Superstructure(m_elevator, m_eeWrist, m_eeRollers,
            m_intakeWrist,
            m_intakeRollers, m_channel, true);

    public RobotContainer() {
        NamedCommands.registerCommand("ScoreL4", m_superstructure.score());
        NamedCommands.registerCommand("ElevateL4", m_superstructure.moveL4());
        NamedCommands.registerCommand("IntakeCoral", m_superstructure.intakeCoralAuto());
        NamedCommands.registerCommand("Stow", m_superstructure.stow());
        for (AutoAlignPosition pos : AutoAlignPosition.values()) {
            NamedCommands.registerCommand("AutoAlign" + pos.toString(),
                    driveUntilPoseAndStall(AutoAlignConstants.REEF_POSITIONS.get(pos)));
        }
        // NamedCommands.registerCommand("ScoreL4", new PrintCommand("Score L4"));
        // NamedCommands.registerCommand("ElevateL4", new PrintCommand("Elevate L4"));
        // NamedCommands.registerCommand("IntakeCoral", new
        // PrintCommand("IntakeCoral"));
        // NamedCommands.registerCommand("Stow", new PrintCommand("Stow"));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureAutoAlignBindings();
        configureBindings();

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    var driveMult = slowModeOn ? DriveConstants.SLOW_MODE_MULT : 1;
                    return drive
                            .withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT * driveMult)
                            .withRotationalDeadband(MaxAngularRate * DriveConstants.DRIVE_DEADBAND_MULT * driveMult)
                            .withVelocityX(-m_leftJoystick.getY() * MaxSpeed * driveMult)
                            .withVelocityY(-m_leftJoystick.getX() * MaxSpeed * driveMult)
                            .withRotationalRate(-m_rightJoystick.getX() * MaxAngularRate * driveMult);
                }));

        m_rightJoystick.button(2)
                .onTrue(Commands.runOnce(() -> slowModeOn = true))
                .onFalse(Commands.runOnce(() -> slowModeOn = false));

        // reset yaw
        m_operatorController.button(8).onTrue(drivetrain.runOnce(() -> {
            System.out.println("Zeroing drive");
            drivetrain.seedFieldCentric();
            drivetrain.resetTranslation(new Translation2d(0, 0));
        }).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);

        // manual commands
        // m_operatorController.x()
        // .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
        // .whileTrue(m_elevator.setManualVoltage(m_operatorController.getLeftY()));
        m_operatorController.x()
                .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
                .whileTrue(Commands.run(() -> {
                    m_elevator.setManualVoltage(m_operatorController.getLeftY()).schedule();
                })).onFalse(Commands.runOnce(() -> {
                    m_elevator.setManualVoltage(0).schedule();
                }));
        m_operatorController.a()
                .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kRightX.value, 0.01))
                .whileTrue(Commands.run(() -> {
                    m_channel.setManualVoltage(m_operatorController.getRightX()).schedule();
                })).onFalse(Commands.runOnce(() -> {
                    m_channel.setManualVoltage(0).schedule();
                }));
        m_operatorController.b()
                .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
                .whileTrue(Commands.run(() -> {
                    m_intakeWrist.setManualVoltage(m_operatorController.getLeftY()).schedule();
                })).onFalse(Commands.runOnce(() -> {
                    m_intakeWrist.setManualVoltage(0).schedule();
                }));
        m_operatorController.b()
                .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kRightX.value, 0.01))
                .whileTrue(Commands.run(() -> {
                    m_intakeRollers.setManualVoltage(m_operatorController.getRightX()).schedule();
                })).onFalse(Commands.runOnce(() -> {
                    m_intakeRollers.setManualVoltage(0).schedule();
                }));
        m_operatorController.y().and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
                .whileTrue(Commands.run(() -> {
                    m_eeWrist.setManualVoltage(m_operatorController.getLeftY()).schedule();
                })).onFalse(Commands.runOnce(() -> {
                    m_eeWrist.setManualVoltage(0).schedule();
                }));
        m_operatorController.y().and(m_operatorController.axisMagnitudeGreaterThan(Axis.kRightX.value, 0.01))
                .whileTrue(Commands.run(() -> {
                    m_eeRollers.setManualVoltage(m_operatorController.getRightX()).schedule();
                })).onFalse(Commands.runOnce(() -> {
                    m_eeRollers.setManualVoltage(0).schedule();
                }));

        m_operatorController.b()
                .and(m_operatorController.leftBumper())
                .onTrue(m_superstructure.ejectIntake());

        m_operatorController.y()
                .and(m_operatorController.leftBumper())
                .onTrue(m_superstructure.ejectEE());

        // zero commands
        m_operatorController.button(7).and(m_operatorController.x())
                .onTrue(m_elevator.zero().ignoringDisable(true));
        m_operatorController.button(7).and(m_operatorController.y())
                .onTrue(m_eeWrist.zero().ignoringDisable(true));
        m_operatorController.button(7).and(m_operatorController.b())
                .onTrue(m_intakeWrist.zero().ignoringDisable(true));

        // gpMode switching
        m_buttonBoard.button(5).onTrue(m_superstructure.switchMode().ignoringDisable(true));

        m_leftJoystick.povLeft().whileTrue(drivetrain.applyRequest(() -> strafeDrive.withSpeeds(new ChassisSpeeds(
                0.0,
                0.2,
                0.0))));

        m_leftJoystick.povRight().whileTrue(drivetrain.applyRequest(() -> strafeDrive.withSpeeds(new ChassisSpeeds(
                0.0,
                -0.2,
                0.0))));

        // scoring commands
        m_rightJoystick.trigger().onTrue(m_superstructure.intake());
        m_leftJoystick.trigger().onTrue(m_superstructure.score());
        m_buttonBoard.button(1).onTrue(m_superstructure.moveL1());
        m_buttonBoard.button(2).onTrue(m_superstructure.moveL2());
        m_buttonBoard.button(3).onTrue(m_superstructure.moveL3());
        m_buttonBoard.button(4).onTrue(m_superstructure.moveL4());

        // stow commands
        m_leftJoystick.button(2).onTrue(m_superstructure.stow());
        m_operatorController.povUp().onTrue(m_superstructure.stow());

        m_leftJoystick.button(3).whileTrue(Commands.select(
                Map.ofEntries(
                        Map.entry(GPMode.Coral,
                                Commands.runOnce(() -> {
                                    driveUntilPoseAndStall(autoAlignPositionSupplier.get()).schedule();
                                })),
                        Map.entry(GPMode.Algae,
                                driveUntilPoseAndStall(AutoAlignConstants.ALGAE_POSITIONS.get(autoAlignPosition)))),
                m_superstructure::getGPMode));

        m_leftJoystick.button(4).onTrue(
                drivetrain.applyRequest(() -> {
                    var driveMult = slowModeOn ? DriveConstants.SLOW_MODE_MULT : 1;
                    return drive
                            .withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT * driveMult)
                            .withRotationalDeadband(MaxAngularRate * DriveConstants.DRIVE_DEADBAND_MULT * driveMult)
                            .withVelocityX(-m_leftJoystick.getY() * MaxSpeed * driveMult)
                            .withVelocityY(-m_leftJoystick.getX() * MaxSpeed * driveMult)
                            .withRotationalRate(-m_rightJoystick.getX() * MaxAngularRate * driveMult);
                }));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void resetMechs() {
        System.out.println("Reseting mechs");
        m_intakeRollers.stop();
        m_intakeWrist.kill();
        m_channel.stop();
        m_elevator.kill();
        m_eeWrist.kill();
        m_eeRollers.stop();
    }

    public void zeroAll() {

    }

    public Command driveUntilPoseAndStall(Pose2d pose) {
        return Commands.sequence(
                new PrintCommand("pose: " + pose.toString()),
                new DriveToPoseCommand(drivetrain, pose),
                new DriveUntilStall(drivetrain));
    }

    public Command ScoreCoral(Pose2d pose) {
        return Commands.sequence(
                driveUntilPoseAndStall(pose),
                m_superstructure.score());
    }

    public void configureAutoAlignBindings() {
        m_buttonBoard.button(6).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to k");
            autoAlignPosition = AutoAlignPosition.K;
        }));
        m_buttonBoard.button(7).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to j");
            autoAlignPosition = AutoAlignPosition.J;
        }));
        m_buttonBoard.button(8).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to i");
            autoAlignPosition = AutoAlignPosition.I;
        }));
        m_buttonBoard.button(9).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to h");
            autoAlignPosition = AutoAlignPosition.H;
        }));
        m_buttonBoard.button(10).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to g");
            autoAlignPosition = AutoAlignPosition.G;
        }));
        m_buttonBoard.button(11).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to f");
            autoAlignPosition = AutoAlignPosition.F;
        }));
        m_buttonBoard.button(12).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to e");
            autoAlignPosition = AutoAlignPosition.E;
        }));
        m_buttonBoard.button(13).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to d");
            autoAlignPosition = AutoAlignPosition.D;
        }));
        m_buttonBoard.button(14).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to c");
            autoAlignPosition = AutoAlignPosition.C;
        }));
        m_buttonBoard.button(15).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to b");
            autoAlignPosition = AutoAlignPosition.B;
        }));
        m_buttonBoard.button(16).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to a");
            autoAlignPosition = AutoAlignPosition.A;
        }));
        m_buttonBoard.button(17).onTrue(Commands.runOnce(() -> {
            // System.out.println("changing to l");
            autoAlignPosition = AutoAlignPosition.L;
        }));
    }
}
