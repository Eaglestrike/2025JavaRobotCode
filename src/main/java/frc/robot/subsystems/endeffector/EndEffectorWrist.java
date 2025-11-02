// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.*;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.superstructure.GPMode;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.*;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;

public class EndEffectorWrist extends SubsystemBase {

	private final TalonFX m_motor = new TalonFX(EndEffectorConstants.Wrist.MOTOR_ID, "rio");
	private final CANcoder m_encoder = new CANcoder(EndEffectorConstants.Wrist.ENCODER_ID, "rio");

	private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
	private boolean debug;

	private boolean dealgaeWhileScoring = true;
	private ElasticSender m_elastic;
	private Superstructure m_superstructure;
	private EndEffectorWristPosition m_currPosition = EndEffectorWristPosition.STOW_ANGLE;
	private EndEffectorWristSide m_currSide = EndEffectorWristSide.FRONT;
	TalonFXConfiguration cfg;

	private EagleSwerveDrivetrain m_drive;

	private Supplier<Pose3d> m_elevatorPoseSupplier;

	public EndEffectorWrist(boolean debug, EagleSwerveDrivetrain drive,
			Supplier<Pose3d> elevatorPoseSupplier) {
		this.m_elevatorPoseSupplier = elevatorPoseSupplier;
		this.debug = debug;
		this.m_drive = drive;
		m_elastic = new ElasticSender("EE: Wrist", debug);
		m_elastic.addButton("Zero", zero());
		m_elastic.addButton("Kill", kill());

		cfg = new TalonFXConfiguration();
		cfg.Feedback.FeedbackRemoteSensorID = EndEffectorConstants.Wrist.ENCODER_ID;
		cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		// cfg.ClosedLoopGeneral.ContinuousWrap = true;

		MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
		motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
		motorConfigs.NeutralMode = NeutralModeValue.Brake;

		FeedbackConfigs fdb = cfg.Feedback;
		fdb.SensorToMechanismRatio = EndEffectorConstants.Wrist.SENSOR_TO_MECHANISM_RATIO;
		fdb.RotorToSensorRatio = EndEffectorConstants.Wrist.ROTOR_TO_SENSOR_RATIO;
		fdb.FeedbackRotorOffset = EndEffectorConstants.Wrist.OFFSET;

		MotionMagicConfigs mm = cfg.MotionMagic;
		mm.MotionMagicCruiseVelocity = EndEffectorConstants.Wrist.MOTION_CRUISE_VELOCITY;
		mm.MotionMagicAcceleration = EndEffectorConstants.Wrist.MOTION_ACCELERATION;
		mm.MotionMagicJerk = EndEffectorConstants.Wrist.MOTION_JERK;

		Slot0Configs slot0 = cfg.Slot0;
		slot0.GravityType = GravityTypeValue.Arm_Cosine;
		slot0.kS = EndEffectorConstants.Wrist.kS;
		slot0.kG = EndEffectorConstants.Wrist.kG;
		slot0.kV = EndEffectorConstants.Wrist.kV;
		slot0.kA = EndEffectorConstants.Wrist.kA;
		slot0.kP = EndEffectorConstants.Wrist.kP;
		slot0.kI = EndEffectorConstants.Wrist.kI;
		slot0.kD = EndEffectorConstants.Wrist.kD;

		m_motor.getConfigurator().apply(cfg);
	}

	public void setSuperstructure(Superstructure superstructure) {
		this.m_superstructure = superstructure;
	}

	public void ElasticInit() {

	}

	public Command moveTo(EndEffectorWristPosition position) {
		return Commands.runOnce(() -> {
			Pose2d currentPose = m_drive.getState().Pose;
			EndEffectorWristSide side = EndEffectorSideUtils.facingReef(currentPose) ? EndEffectorWristSide.FRONT
					: EndEffectorWristSide.BACK;
			// EndEffectorWristSide side = EndEffectorWristSide.BACK;
			moveTo(position, side).schedule();
		});
	}

	public Command moveTo(EndEffectorWristPosition position, EndEffectorWristSide side) {
		return runOnce(
				() -> {
					m_currPosition = position;
					m_currSide = side;
					double angle = position.getAngle();
					if (side == EndEffectorWristSide.BACK && m_superstructure != null && m_superstructure.getGPMode() == GPMode.Coral) {
						angle = position.getBackAngle();
					}
					m_motor.setControl(m_mmReq.withPosition(angle).withSlot(0));
				});
	}

	public void moveToFunc(EndEffectorWristPosition position, EndEffectorWristSide side) {

		m_currPosition = position;
		m_currSide = side;
		double angle = position.getAngle();
		if (side == EndEffectorWristSide.BACK) {
			angle = position.getBackAngle();
		}
		m_motor.setControl(m_mmReq.withPosition(angle).withSlot(0));

	}

	public void moveToNextPosition() {
		moveToNextPosition(m_currSide);
	}

	public void moveToNextPosition(EndEffectorWristSide side) {
		EndEffectorWristPosition nextPosition = EndEffectorWristPosition.STOW_ANGLE;
		switch (m_currPosition) {
			case L1_SCORE_ANGLE:
				nextPosition = EndEffectorWristPosition.STOW_ANGLE;
				break;
			case L2_PRE_ANGLE:
				nextPosition = EndEffectorWristPosition.L2_SCORE_ANGLE;
				break;
			case L3_PRE_ANGLE:
				nextPosition = EndEffectorWristPosition.L3_SCORE_ANGLE;
				break;
			case L4_PRE_ANGLE:
				nextPosition = dealgaeWhileScoring ? EndEffectorWristPosition.L4_SCORE_ANGLE
						: EndEffectorWristPosition.L4_SCORE_NO_DEALGAE_ANGLE;
				break;
			case SCORE_PROCESSOR_ANGLE:
			case SCORE_BARGE_ANGLE:
			case INTAKE_ALGAE_ANGLE_L2:
			case INTAKE_ALGAE_ANGLE_L3:
				nextPosition = EndEffectorWristPosition.STOW_ANGLE;
				break;
			case L2_SCORE_ANGLE:
			case L3_SCORE_ANGLE:
			case L4_SCORE_ANGLE:
			case L4_SCORE_NO_DEALGAE_ANGLE:
				nextPosition = m_currPosition;
				break;
			default:
				nextPosition = EndEffectorWristPosition.STOW_ANGLE;
				break;
		}
		moveToFunc(nextPosition, side);
	}

	public Command zero() {
		return runOnce(
				() -> {
					System.out.println("Zeroing End Effector");
					m_encoder.setPosition(EndEffectorConstants.Wrist.OFFSET);
				});
	}

	public Command kill() {
		return runOnce(
				() -> {
					m_motor.disable();
				});
	}

	public Command setManualVoltage(double joystickPosition) {
		return run(
				() -> {
					m_motor.setVoltage(joystickPosition * EndEffectorConstants.Wrist.MAX_VOLTS
							/ EndEffectorConstants.Wrist.MANUAL_RATIO);
				});
	}

	public EndEffectorWristPosition getPosition() {
		return m_currPosition;
	}

	/**
	 * Sets whether to dealgae while scoring
	 * 
	 * @param dealgae <code>true</code> to dealgae while scoring, <code>false</code>
	 *                to not
	 */
	public void setDealgae(boolean dealgae) {
		this.dealgaeWhileScoring = dealgae;
	}

	@Override
	public void periodic() {
		m_elastic.periodic();
	}

	private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
			DCMotor.getKrakenX60(1),
			58.56, // motor (shaft to) 9t (connected to) 84t (shaft to) 18t (connected to) 64t
					// (shaft to) 34t (connected to) 60t
			SingleJointedArmSim.estimateMOI(0.578228, 3.17),
			0.578228,
			Double.MIN_VALUE,
			Double.MAX_VALUE,
			true,
			0);

	@AutoLogOutput
	public Pose3d getPose() {
		return m_elevatorPoseSupplier.get()
				.plus(new Transform3d(new Translation3d(0, 0.2, 0.9),
						new Rotation3d(0, -m_motor.getPosition().getValueAsDouble(), 0)));
	}

	@Override
	public void simulationPeriodic() {
		var talonFXSim = m_motor.getSimState();
		var canCoderSim = m_encoder.getSimState();
		talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
		var motorVoltage = talonFXSim.getMotorVoltageMeasure();
		m_armSim.setInputVoltage(motorVoltage.in(Volts));
		m_armSim.update(0.02);
		// System.out.println(m_armSim.getAngleRads());
		canCoderSim.setRawPosition(m_armSim.getAngleRads() * EndEffectorConstants.Wrist.SENSOR_TO_MECHANISM_RATIO);
		canCoderSim.setVelocity(m_armSim.getVelocityRadPerSec() * EndEffectorConstants.Wrist.SENSOR_TO_MECHANISM_RATIO);
	}
}
