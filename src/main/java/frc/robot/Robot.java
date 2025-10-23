package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import au.grapplerobotics.CanBridge;
import badgerlog.Dashboard;
import badgerlog.DashboardConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  StructPublisher<Pose2d> zeroedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("DrivebaseZeroedPose", Pose2d.struct).publish();
  StructArrayPublisher<Pose3d> mechPosePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MechanismPosesZeroed", Pose3d.struct).publish();

 
  public Robot() {
    Dashboard.configure(DashboardConfig.defaultConfig);
    CanBridge.runTCP();
    Logger.recordMetadata("ProjectName", "Theseus");
    Logger.addDataReceiver(new NT4Publisher());

    Logger.start();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    zeroedPosePublisher.set(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    mechPosePublisher.set(new Pose3d[] {
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)),
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)),
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0))
    });
    Dashboard.update();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getEEWrist().setDealgae(true).schedule();
    m_robotContainer.resetMechs();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
