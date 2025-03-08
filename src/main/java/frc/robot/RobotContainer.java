// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final Drive m_drive = new Drive(new File(Filesystem.getDeployDirectory(),
			"swerve"), DriveConstants.Positions.kStartingPose);
	private final Elevator m_elevator = new Elevator();
	private final AlgaeArm m_algaeArm = new AlgaeArm();
	private final AlgaeIntake m_algaeIntake = new AlgaeIntake();

	// The driver's controller
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandJoystick m_operatorsStick = new CommandJoystick(OIConstants.kOperatorStickPort);

	private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_autonomousChooser.addOption("do nothing", new InstantCommand());
		m_autonomousChooser.setDefaultOption("go back",
				m_drive.driveCommand(() -> -1, () -> 0, () -> 0, () -> -1).until(
						() -> m_drive.getPose().getTranslation()
								.getDistance(DriveConstants.Positions.kStartingPose.getTranslation()) > 1));
		SmartDashboard.putData(m_autonomousChooser);

		configureButtonBindings();

		m_drive.setDefaultCommand(driveFieldOrientedDirectAngle());
		m_elevator.setDefaultCommand(m_elevator.idleCommand());
		m_algaeArm.setDefaultCommand(m_algaeArm.upCommand());
		m_algaeIntake.setDefaultCommand(m_algaeIntake.idleCommand());
	}

	private Command driveFieldOrientedDirectAngle() {
		return m_drive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kTranslationX),
						OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(
						OIConstants.kTranslationY), OIConstants.kDeadband),
				() -> -m_driverController.getRawAxis(OIConstants.kHeadingX),
				() -> -m_driverController.getRawAxis(OIConstants.kHeadingY));
	}

	private Command driveFieldOrientedInverseDirectAngle() {
		return m_drive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kTranslationX),
						OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(
						OIConstants.kTranslationY), OIConstants.kDeadband),
				() -> m_driverController.getRawAxis(OIConstants.kHeadingX),
				() -> m_driverController.getRawAxis(OIConstants.kHeadingY));
	}

	@SuppressWarnings("unused")
	private Command driveFieldOriented() {
		return m_drive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kTranslationX),
						OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(
						OIConstants.kTranslationY), OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(
						m_driverController.getRawAxis(OIConstants.kRotation), OIConstants.kDeadband));
	}

	private Command lockToHeading(Rotation2d heading) {
		return m_drive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kTranslationX),
						OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(
						OIConstants.kTranslationY), OIConstants.kDeadband),
				() -> heading);
	}

	private Command pickupAlgae() {
		return m_algaeArm.downCommand().raceWith(m_algaeIntake.inCommand());
	}

	private void configureButtonBindings() {
		m_operatorsStick.button(OIConstants.kScoreAlgae).whileTrue(m_algaeIntake.outCommand());
		m_operatorsStick.button(OIConstants.kIntakeAlgae).whileTrue(pickupAlgae());

		m_driverController.button(OIConstants.kZeroGyro).onTrue(new InstantCommand(m_drive::zeroGyro));
		m_driverController.button(OIConstants.kIntakeLeft)
				.whileTrue(lockToHeading(new Rotation2d(DriveConstants.Positions.kLeftIntakeHeading)));
		m_driverController.button(OIConstants.kIntakeRight)
				.whileTrue(lockToHeading(new Rotation2d(DriveConstants.Positions.kRightIntakeHeading)));
		m_driverController.leftTrigger()
				.whileTrue(lockToHeading(new Rotation2d(DriveConstants.Positions.kProcessorHeading)));
		m_driverController.rightTrigger().whileTrue(driveFieldOrientedInverseDirectAngle());

		// m_driverController.button(OIConstants.kSlowMode).onTrue(m_robotDrive.setSlowModeCommand(true))
		// .onFalse(m_robotDrive.setSlowModeCommand(false));
		// m_driverController.button(OIConstants.kRobotRelative).onTrue(m_robotDrive.setFieldRelativeCommand(false))
		// .onFalse(m_robotDrive.setFieldRelativeCommand(true));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return m_autonomousChooser.getSelected();
	}
}
