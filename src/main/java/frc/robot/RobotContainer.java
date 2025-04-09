// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.GroundAlgaeArm.GroundAlgaeArm;
import frc.robot.subsystems.GroundAlgaeIntake.GroundAlgaeIntake;
import frc.robot.subsystems.Elevator.Elevator;
// import frc.robot.subsystems.AlgaeStick.AlgaeStick;
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
	// private final AlgaeStick algaeStick = new AlgaeStick();

	// The driver's controller
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandJoystick m_operatorsStick = new CommandJoystick(OIConstants.kOperatorStickPort);

	// private final SendableChooser<Command> m_autonomousChooser = new
	// SendableChooser<>();
	private final SendableChooser<Command> autoChooser;

	private final Command driveToProcessor;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Autonomous", autoChooser);

		// m_autonomousChooser.addOption("do nothing", new InstantCommand());
		// m_autonomousChooser.setDefaultOption("go back",
		// m_drive.driveCommand(() -> -1, () -> 0, () -> 0, () -> 1).until(
		// () -> m_drive.getPose().getTranslation()
		// .getDistance(DriveConstants.Positions.kStartingPose.getTranslation()) > 1));
		// SmartDashboard.putData("Autonomous", m_autonomousChooser);

		Command _driveToProcessor;
		try {
			// Load the path we want to pathfind to and follow
			PathPlannerPath path = PathPlannerPath.fromPathFile("Approach Processor");
			// Create the constraints to use while pathfinding. The constraints defined in
			// the path will only be used for the path.
			PathConstraints constraints = new PathConstraints(
					3.0, 4.0,
					Units.degreesToRadians(540), Units.degreesToRadians(720));

			// Since AutoBuilder is configured, we can use it to build pathfinding commands
			_driveToProcessor = AutoBuilder.pathfindThenFollowPath(
					path,
					constraints);
		} catch (FileVersionException | IOException | ParseException e) {
			// TODO Auto-generated catch block
			_driveToProcessor = Commands.none();
			e.printStackTrace();
		}
		driveToProcessor = _driveToProcessor;

		configureButtonBindings();

		m_drive.setDefaultCommand(driveFieldOrientedInverseDirectAngle());
		m_elevator.setDefaultCommand(m_elevator.stopCommand());
		m_algaeArm.setDefaultCommand(m_algaeArm.upCommand());
		m_algaeIntake.setDefaultCommand(m_algaeIntake.idleCommand());
		// algaeStick.setDefaultCommand(algaeStick.highCommand());
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

	private Command driveFieldOrientedStickDirectAngle() {
		return m_drive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kTranslationX),
						OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(
						OIConstants.kTranslationY), OIConstants.kDeadband),
				() -> -m_driverController.getRawAxis(OIConstants.kHeadingY),
				() -> m_driverController.getRawAxis(OIConstants.kHeadingX));
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

	private Command lockToAlgae() {
		final Supplier<Rotation2d> heading = () -> m_drive.getHeading()
				.minus(Rotation2d.fromDegrees(LimelightHelpers.getTX(LimelightConstants.kLimelightName)));
		return m_drive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kTranslationX),
						OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(
						OIConstants.kTranslationY), OIConstants.kDeadband),
				heading);
	}

	private Command trackAlgae() {
		return m_drive.run(() -> {
			if (LimelightHelpers.getTargetCount(LimelightConstants.kLimelightName) > 0) {
				lockToAlgae().execute();
			} else {
				m_drive.getDefaultCommand().execute();
			}
		});
	}

	private Command pickupAlgae() {
		return m_algaeArm.downCommand().raceWith(m_algaeIntake.inCommand());
	}

	private Command climbMode() {
		return m_algaeArm.downCommand();
	}

	private void configureButtonBindings() {
		m_operatorsStick.button(OIConstants.kScoreAlgae).whileTrue(m_algaeIntake.outCommand());
		m_operatorsStick.button(OIConstants.kIntakeAlgae).whileTrue(pickupAlgae());
		// m_operatorsStick.button(2).whileTrue(algaeStick.lowCommand());
		m_operatorsStick.button(2).whileTrue(m_elevator.climbCommand().alongWith(climbMode()));
		m_operatorsStick.button(5).whileTrue(m_elevator.downCommand());
		m_operatorsStick.button(6).whileTrue(m_elevator.upCommand());
		m_operatorsStick.button(11).whileTrue(climbMode());

		m_driverController.button(OIConstants.kZeroGyro).onTrue(new InstantCommand(m_drive::zeroGyro180));
		m_driverController.rightTrigger()
				.whileTrue(lockToHeading(new Rotation2d(DriveConstants.Positions.kProcessorHeading)));
		m_driverController.leftTrigger().whileTrue(driveFieldOrientedStickDirectAngle());
		m_driverController.rightBumper().whileTrue(trackAlgae());
		m_driverController.a().whileTrue(m_drive.run(m_drive::lock));
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
		// m_driverController.leftTrigger().and(DriverStation::isTest).whileTrue(algaeStick.lowCommand());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
