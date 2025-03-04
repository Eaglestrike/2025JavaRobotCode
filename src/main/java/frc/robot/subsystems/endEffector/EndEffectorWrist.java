// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import frc.robot.constants.EndEffectorConstants;
import frc.robot.utils.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class EndEffectorWrist extends SubsystemBase {

	private final TalonFX m_fx = new TalonFX(EndEffectorConstants.Wrist.MOTOR_ID, "rio");
	private final TalonFXConfiguration cfg = new TalonFXConfiguration();
	private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
	private boolean debug;
	private ElasticSender m_elastic;

	public EndEffectorWrist(boolean debug) {
		this.debug = debug;
		m_elastic = new ElasticSender("EE: Wrist", debug);

		FeedbackConfigs fdb = cfg.Feedback;
		fdb.FeedbackRemoteSensorID = EndEffectorConstants.Wrist.ENCODER_ID;
		fdb.SensorToMechanismRatio = EndEffectorConstants.Wrist.GEAR_RATIO;
		fdb.FeedbackRotorOffset = EndEffectorConstants.Wrist.OFFSET;

		MotionMagicConfigs mm = cfg.MotionMagic;
		mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
				.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
				.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

		Slot0Configs slot0 = cfg.Slot0;
		slot0.GravityType = GravityTypeValue.Arm_Cosine;
		slot0.kS = EndEffectorConstants.Wrist.kS;
		slot0.kG = EndEffectorConstants.Wrist.kG;
		slot0.kV = EndEffectorConstants.Wrist.kV;
		slot0.kA = EndEffectorConstants.Wrist.kA;
		slot0.kP = EndEffectorConstants.Wrist.kP;
		slot0.kI = EndEffectorConstants.Wrist.kI;
		slot0.kD = EndEffectorConstants.Wrist.kD;

		m_fx.getConfigurator().apply(cfg);
	}

	public void ElasticInit() {

	}

	public Command moveTo(double position) {
		return runOnce(
				() -> {
					m_fx.setControl(m_mmReq.withPosition(position).withSlot(0));
				});
	}

	public Command zero() {
		return runOnce(
				() -> {
					m_fx.setPosition(EndEffectorConstants.Wrist.OFFSET);
				});
	}

	public Command kill() {
		return runOnce(
				() -> {
					m_fx.disable();
				});
	}

	public Command setManualVoltage(double joystickPosition) {
		return run(
				() -> {
					m_fx.setVoltage(joystickPosition * EndEffectorConstants.Wrist.MAX_VOLTS
							/ EndEffectorConstants.Wrist.MANUAL_RATIO);
				});
	}

	@Override
	public void periodic() {
		if (debug) {
			m_elastic.update();
		}
	}
}
