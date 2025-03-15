// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.*;
import frc.robot.utils.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

public class EndEffectorWrist extends SubsystemBase {

	private final TalonFX m_motor = new TalonFX(EndEffectorConstants.Wrist.MOTOR_ID, "rio");
	private final CANcoder m_encoder = new CANcoder(EndEffectorConstants.Wrist.ENCODER_ID, "rio");

	private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
	private boolean debug;
	private ElasticSender m_elastic;
	private EndEffectorWristPosition m_currPosition = EndEffectorWristPosition.STOW_ANGLE;
	private EndEffectorWristSide m_currSide = EndEffectorWristSide.FRONT;
	TalonFXConfiguration cfg;

	public EndEffectorWrist(boolean debug) {
		this.debug = debug;
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

	public void ElasticInit() {

	}

	public Command moveTo(EndEffectorWristPosition position) {
		return moveTo(position, EndEffectorWristSide.FRONT);
	}

	public void setBargeSpeed(boolean on) {
		if (on) {
			cfg.MotionMagic.MotionMagicAcceleration = 40;
			cfg.MotionMagic.MotionMagicCruiseVelocity = 40;
			m_motor.getConfigurator().apply(cfg);
		} else {
			cfg.MotionMagic.MotionMagicAcceleration = 20;
			cfg.MotionMagic.MotionMagicCruiseVelocity = 20;
			m_motor.getConfigurator().apply(cfg);
		}
	}

	public Command moveTo(EndEffectorWristPosition position, EndEffectorWristSide side) {
		return runOnce(
				() -> {
					System.out.println("moving to " + position.getAngle());
					m_currPosition = position;
					m_currSide = side;
					double angle = position.getAngle();
					// if (side == EndEffectorWristSide.BACK) {
					// angle = position.getBackAngle();
					// }
					m_motor.setControl(m_mmReq.withPosition(angle).withSlot(0));
				});
	}

	public void moveToFunc(EndEffectorWristPosition position, EndEffectorWristSide side) {

		System.out.println("moving to " + position.getAngle());
		m_currPosition = position;
		m_currSide = side;
		double angle = position.getAngle();
		// if (side == EndEffectorWristSide.BACK) {
		// angle = position.getBackAngle();
		// }
		m_motor.setControl(m_mmReq.withPosition(angle).withSlot(0));

	}

	public void moveToNextPosition() {
		System.out.println("calling next func ");
		moveToNextPosition(m_currSide);
	}

	public void moveToNextPosition(EndEffectorWristSide side) {
		System.out.println("finding pos ");
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
				nextPosition = EndEffectorWristPosition.L4_SCORE_ANGLE;
				break;
			case SCORE_PROCESSOR_ANGLE:
			case SCORE_BARGE_ANGLE:
			case INTAKE_ALGAE_ANGLE:
				nextPosition = EndEffectorWristPosition.STOW_ANGLE;
				break;
			case L2_SCORE_ANGLE:
			case L3_SCORE_ANGLE:
			case L4_SCORE_ANGLE:
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

	@Override
	public void periodic() {
		m_elastic.periodic();
	}
}
