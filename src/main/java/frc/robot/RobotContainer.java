// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.constants.AutoAlignPosition;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.ReefPositions;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.Drive.EagleSwerveTelemetry;
import frc.robot.subsystems.channel.Channel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffectorRollers;
import frc.robot.subsystems.endeffector.EndEffectorSideUtils;
import frc.robot.subsystems.endeffector.EndEffectorWrist;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.superstructure.GPMode;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // swerve
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT)
            .withRotationalDeadband(MaxAngularRate * DriveConstants.DRIVE_DEADBAND_MULT)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.ApplyRobotSpeeds strafeDrive = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final EagleSwerveTelemetry logger = new EagleSwerveTelemetry(MaxSpeed);
    public final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision(drivetrain);

    private boolean slowModeOn = false;
    private AutoAlignPosition autoAlignPosition = AutoAlignPosition.A;
    public Supplier<Pose2d> autoAlignPositionSupplier = () -> {
        Pose2d reefPosition = ReefPositions.getReefPosition(DriverStation.getAlliance().orElse(Alliance.Blue),
                autoAlignPosition);
        reefPosition = new Pose2d(reefPosition.getTranslation(),
                reefPosition.getRotation().rotateBy(new Rotation2d(Math.PI)));
        if (EndEffectorSideUtils.facingReef(drivetrain.getState().Pose)) {
            // System.out.println("[Auto Align] Flipping reef position, not facing reef");
            reefPosition = new Pose2d(reefPosition.getTranslation(),
                    reefPosition.getRotation().rotateBy(new Rotation2d(Math.PI)));
        }
        return reefPosition;
    };

    // controllers
    private final CommandJoystick m_leftJoystick = new CommandJoystick(0);
    private final CommandJoystick m_rightJoystick = new CommandJoystick(1);
    private final CommandXboxController m_operatorController = new CommandXboxController(2);
    private final CommandXboxController m_buttonBoard = new CommandXboxController(3);

    // auto
    private final SendableChooser<Command> autoChooser;

    private final Elevator m_elevator = new Elevator(true);
    private final EndEffectorRollers m_eeRollers = new EndEffectorRollers(drivetrain);
    private final EndEffectorWrist m_eeWrist = new EndEffectorWrist(true, drivetrain, m_elevator::getPose);
    private final IntakeWrist m_intakeWrist = new IntakeWrist(true);
    private final IntakeRollers m_intakeRollers = new IntakeRollers();
    private final Channel m_channel = new Channel(true);
    private final Superstructure m_superstructure = new Superstructure(m_elevator, m_eeWrist, m_eeRollers,
            m_intakeWrist,
            m_intakeRollers, m_channel, true);


    // suppliers
    private Supplier<GPMode> gpModeSupplier = () -> m_superstructure.getGPMode();
    private BooleanSupplier algaeModeSupplier = () -> m_superstructure.getGPMode() == GPMode.Algae;
    private BooleanSupplier coralModeSupplier = () -> m_superstructure.getGPMode() == GPMode.Coral;

    public RobotContainer() {
        NamedCommands.registerCommand("ScoreL4", m_superstructure.score());
        NamedCommands.registerCommand("OuttakeL4",
                m_eeRollers.run(EndEffectorConstants.Rollers.OUTTAKE_L4_CORAL_VOLTS));
        NamedCommands.registerCommand("StopRollers", m_eeRollers.fullStop());
        NamedCommands.registerCommand("ElevateL4", m_superstructure.moveL4());
        NamedCommands.registerCommand("IntakeCoral", m_superstructure.intake());
        NamedCommands.registerCommand("Stow", m_superstructure.stow());
        NamedCommands.registerCommand("SetDealgaeFalse", Commands.runOnce(() -> m_eeWrist.setDealgae(false)));
        for (AutoAlignPosition pos : AutoAlignConstants.REEF_POSITIONS.keySet()) {
            NamedCommands.registerCommand("AutoAlign" + pos.toString(),
                    drivetrain.alignPID(() -> ReefPositions
                            .getReefPosition(DriverStation.getAlliance().orElse(Alliance.Blue), pos)));
        }

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

        // Outtake to Score if stuck
        m_leftJoystick.povLeft()
                .whileTrue(m_eeRollers.run(EndEffectorConstants.Rollers.OUTTAKE_L4_CORAL_VOLTS))
                .onFalse(m_eeRollers.fullStop());

        m_leftJoystick.povRight()
                .whileTrue(m_eeRollers.run(EndEffectorConstants.Rollers.OUTTAKE_L4_CORAL_VOLTS))
                .onFalse(m_eeRollers.fullStop());

        m_leftJoystick.povUp()
                .whileTrue(m_eeRollers.run(EndEffectorConstants.Rollers.OUTTAKE_L4_CORAL_VOLTS))
                .onFalse(m_eeRollers.fullStop());

        m_leftJoystick.povDown()
                .whileTrue(m_eeRollers.run(EndEffectorConstants.Rollers.OUTTAKE_L4_CORAL_VOLTS))
                .onFalse(m_eeRollers.fullStop());

        // scoring commands
        m_rightJoystick.trigger().onTrue(m_superstructure.intake()).debounce(0.25);
        m_leftJoystick.trigger().onTrue(m_superstructure.score());
        m_buttonBoard.button(1).onTrue(m_superstructure.moveL1());
        m_buttonBoard.button(2).onTrue(m_superstructure.moveL2());
        m_buttonBoard.button(3).onTrue(m_superstructure.moveL3());
        m_buttonBoard.button(4).onTrue(m_superstructure.moveL4());

        // stow commands
        m_leftJoystick.button(2).onTrue(m_superstructure.stow());
        m_operatorController.povUp().onTrue(m_superstructure.stow());

        m_leftJoystick.button(3).and(coralModeSupplier)
                .whileTrue(Commands.sequence(
                        drivetrain.alignPID(autoAlignPositionSupplier),
                        new WaitCommand(0.5),
                        m_superstructure.score()));

        m_leftJoystick.button(3).and(algaeModeSupplier)
                .whileTrue(Commands.sequence(
                        new ShootBarge(drivetrain, m_superstructure),
                        m_superstructure.score()));

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

    // public Command driveUntilPoseAndStall(Supplier<Pose2d> pose) {
    // return Commands.sequence(
    // new PrintCommand("pose: " + pose.toString()),
    // new DriveToPosePID(drivetrain, pose.get()),
    // new DriveUntilStall(drivetrain));
    // }

    public Command driveUntilPoseAndStall(Pose2d pose) {
        return Commands.sequence(
                new PrintCommand("pose: " + pose.toString()),
                new DriveToPosePID(drivetrain, pose)
        // new DriveUntilStall(drivetrain)
        );
    }

    public EndEffectorWrist getEEWrist() {
        return m_eeWrist;
    }

    public void configureAutoAlignBindings() {
        m_buttonBoard.button(6).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.K;
        }));
        m_buttonBoard.button(7).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.J;
        }));
        m_buttonBoard.button(8).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.I;
        }));
        m_buttonBoard.button(9).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.H;
        }));
        m_buttonBoard.button(10).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.G;
        }));
        m_buttonBoard.button(11).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.F;
        }));
        m_buttonBoard.button(12).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.E;
        }));
        m_buttonBoard.button(13).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.D;
        }));
        m_buttonBoard.button(14).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.C;
        }));
        m_buttonBoard.button(15).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.B;
        }));
        m_buttonBoard.button(16).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.A;
        }));
        m_buttonBoard.button(17).and(coralModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.L;
        }));

        m_buttonBoard.button(6).or(m_buttonBoard.button(17)).and(algaeModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.ALGAE_KL;
        }));

        m_buttonBoard.button(7).or(m_buttonBoard.button(8)).and(algaeModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.ALGAE_IJ;
        }));

        m_buttonBoard.button(9).or(m_buttonBoard.button(10)).and(algaeModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.ALGAE_GH;
        }));

        m_buttonBoard.button(11).or(m_buttonBoard.button(12)).and(algaeModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.ALGAE_EF;
        }));

        m_buttonBoard.button(13).or(m_buttonBoard.button(14)).and(algaeModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.ALGAE_CD;
        }));

        m_buttonBoard.button(15).or(m_buttonBoard.button(16)).and(algaeModeSupplier).onTrue(Commands.runOnce(() -> {
            autoAlignPosition = AutoAlignPosition.ALGAE_AB;
        }));
    }
}
