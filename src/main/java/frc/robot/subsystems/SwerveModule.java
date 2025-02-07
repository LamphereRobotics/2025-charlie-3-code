// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
	private final SparkMax m_driveMotor;
	private final SparkMax m_turningMotor;

	private final CANcoder m_turningEncoder;
	private final RelativeEncoder m_driveEncoder;

	private final PIDController m_drivePIDController = new PIDController(
			ModuleConstants.kPModuleDriveController,
			ModuleConstants.kIModuleDriveController, ModuleConstants.kDModuleDriveController);

	private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.kSModuleDriveFeedforward, ModuleConstants.kVModuleDriveFeedforward);
	private final SimpleMotorFeedforward m_turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.kSModuleTurningFeedforward, ModuleConstants.kVModuleTurningFeedforward);

	// Using a TrapezoidProfile PIDController to allow for smooth turning
	private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
			ModuleConstants.kPModuleTurningController,
			ModuleConstants.kIModuleTurningController,
			ModuleConstants.kDModuleTurningController,
			new TrapezoidProfile.Constraints(
					ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
					ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

	/**
	 * Constructs a SwerveModule.m_drivePIDController
	 *
	 * @param driveMotorChannel      The channel of the drive motor.
	 * @param turningMotorChannel    The channel of the turning motor.
	 * @param turningEncoderChannel  The channel of the turning encoder.
	 * @param turningEncoderReversed Whether the turning encoder is reversed.
	 */
	public SwerveModule(
			int driveMotorChannel,
			int turningMotorChannel,
			int turningEncoderChannel,
			boolean turningEncoderReversed) {
		m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
		m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
		
		SparkMaxConfig driveMotorConfig_ = new SparkMaxConfig();
		SparkMaxConfig turnMotorConfig_ = new SparkMaxConfig();

		driveMotorConfig_.idleMode(SparkBaseConfig.IdleMode.kBrake);
		driveMotorConfig_.smartCurrentLimit(ModuleConstants.kMaxDriveCurrent);
		driveMotorConfig_.encoder.positionConversionFactor(Constants.DriveConstants.kDriveScale);
		driveMotorConfig_.encoder.velocityConversionFactor(Constants.DriveConstants.kDriveScale / 60);
		turnMotorConfig_.idleMode(SparkBaseConfig.IdleMode.kBrake);
		turnMotorConfig_.smartCurrentLimit(ModuleConstants.kMaxTurnCurrent);
		turnMotorConfig_.inverted(true);
		m_driveMotor.configure(driveMotorConfig_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
		m_turningMotor.configure(turnMotorConfig_, null, null);

		m_turningEncoder = new CANcoder(turningEncoderChannel);

		m_driveEncoder = m_driveMotor.getEncoder();	
		m_drivePIDController.setTolerance(Constants.ModuleConstants.kToleranceModuleDriveController);

		// Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

		m_turningPIDController.setIZone(Constants.ModuleConstants.kIZoneModuleTurningController);
		m_turningPIDController.setTolerance(Constants.ModuleConstants.kPositionToleranceModuleTurningController,
				Constants.ModuleConstants.kVelocityToleranceModuleTurningController);
		m_turningPIDController.setIntegratorRange(-Constants.ModuleConstants.kIntegratorMaxModuleTurningController,
				Constants.ModuleConstants.kIntegratorMaxModuleTurningController);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(
				m_driveEncoder.getVelocity(), new Rotation2d(getTurnAngle()));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
				m_driveEncoder.getPosition(), new Rotation2d(getTurnAngle()));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		var encoderRotation = new Rotation2d(getTurnAngle());

		// Optimize the reference state to avoid spinning further than 90
		desiredState.optimize(encoderRotation);

		// Scale speed by cosine of angle error. This scales down movement perpendicular
		// to the desired
		// direction of travel that can occur when modules change directions. This
		// results in smoother
		// driving.
		desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos();

		// Calculate the drive output from the drive PID controller.
		final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
				desiredState.speedMetersPerSecond);

		// Calculate the turning motor output from the turning PID controller.
		final double turnOutput = m_turningPIDController.calculate(getTurnAngle(),
				desiredState.angle.getRadians());

		// Calculate the turning motor output from the turning PID controller.
		m_driveMotor.setVoltage(
				driveOutput + m_driveFeedForward.calculate(desiredState.speedMetersPerSecond));
		m_turningMotor
				.setVoltage(turnOutput + m_turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity));
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		m_driveEncoder.setPosition(0);
		m_turningEncoder.setPosition(0);
	}

	private double getTurnAngle() {
		return m_turningEncoder.getAbsolutePosition().getValue().in(Radians);
	}
	public double getTurnVelocity() {
		return m_turningEncoder.getVelocity().getValue().in(RadiansPerSecond);
	}

	public void logStateToDashboard(String name) {
		SmartDashboard.putNumber(name + "-voltage",
				m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage());
		SmartDashboard.putNumber(name + "-speed",
				getState().speedMetersPerSecond);
		SmartDashboard.putNumber(name + "-turn-angle",
				getState().angle.getDegrees());
		SmartDashboard.putNumber(name + "-turn-velocity",
				getTurnVelocity());
		SmartDashboard.putNumber(name + "-desired-drive-speed",
				m_drivePIDController.getSetpoint());
		SmartDashboard.putNumber(name + "-desired-turn-angle",
				m_turningPIDController.getSetpoint().position);
		SmartDashboard.putNumber(name + "-desired-turn-velocity",
				m_turningPIDController.getSetpoint().velocity);
	}
}
