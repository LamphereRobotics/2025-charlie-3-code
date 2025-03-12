// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.GroundAlgaeArm.GroundAlgaeArm;
import frc.robot.subsystems.GroundAlgaeIntake.GroundAlgaeIntake;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.AlgaeStick.AlgaeStick;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.DriveConstants;

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
	private final GroundAlgaeArm m_algaeArm = new GroundAlgaeArm();
	private final GroundAlgaeIntake m_algaeIntake = new GroundAlgaeIntake();
	private final AlgaeStick algaeStick = new AlgaeStick();

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
				m_drive.driveCommand(() -> -1, () -> 0, () -> 0, () -> 1).until(
						() -> m_drive.getPose().getTranslation()
								.getDistance(DriveConstants.Positions.kStartingPose.getTranslation()) > 1));
		SmartDashboard.putData("Autonomous", m_autonomousChooser);

		configureButtonBindings();

		m_drive.setDefaultCommand(driveFieldOrientedInverseDirectAngle());
		m_elevator.setDefaultCommand(m_elevator.stopCommand());
		m_algaeArm.setDefaultCommand(m_algaeArm.upCommand());
		m_algaeIntake.setDefaultCommand(m_algaeIntake.idleCommand());
		algaeStick.setDefaultCommand(algaeStick.highCommand());
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
		m_operatorsStick.button(2).whileTrue(algaeStick.lowCommand());
		m_operatorsStick.button(5).whileTrue(m_elevator.downCommand());
		m_operatorsStick.button(6).whileTrue(m_elevator.upCommand());

		m_driverController.button(OIConstants.kZeroGyro).onTrue(new InstantCommand(m_drive::zeroGyro180));
		m_driverController.rightTrigger()
				.whileTrue(lockToHeading(new Rotation2d(DriveConstants.Positions.kProcessorHeading)));
		// TODO: create drive slow mode
		// m_driverController.button(OIConstants.kSlowMode).onTrue(m_robotDrive.setSlowModeCommand(true))
		// .onFalse(m_robotDrive.setSlowModeCommand(false));
		// TODO: create robot relative control
		// m_driverController.button(OIConstants.kRobotRelative).onTrue(m_robotDrive.setFieldRelativeCommand(false))
		// .onFalse(m_robotDrive.setFieldRelativeCommand(true));

		m_driverController.rightBumper().and(DriverStation::isTest).whileTrue(m_algaeIntake.outCommand());
		m_driverController.leftBumper().and(DriverStation::isTest).whileTrue(pickupAlgae());
		m_driverController.y().and(DriverStation::isTest).whileTrue(m_elevator.upCommand());
		m_driverController.a().and(DriverStation::isTest).whileTrue(m_elevator.downCommand());
		m_driverController.leftTrigger().and(DriverStation::isTest).whileTrue(algaeStick.lowCommand());
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
