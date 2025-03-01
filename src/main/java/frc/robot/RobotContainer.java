// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final Pose2d m_startingPosition = new Pose2d(0, 0, new Rotation2d(Degrees.of(180)));
	private final Drive m_drive = new Drive(new File(Filesystem.getDeployDirectory(),
			"swerve"), m_startingPosition);
	private final Elevator m_elevator = new Elevator();
	private final CoralIntake m_coralIntake = new CoralIntake();
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
						() -> m_drive.getPose().getTranslation().getDistance(m_startingPosition.getTranslation()) > 1));
		SmartDashboard.putData(m_autonomousChooser);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		Command driveFieldOrientedDirectAngle = m_drive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kTranslationX),
						OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(
						OIConstants.kTranslationY), OIConstants.kDeadband),
				() -> -m_driverController.getRawAxis(OIConstants.kHeadingX),
				() -> -m_driverController.getRawAxis(OIConstants.kHeadingY));
		@SuppressWarnings("unused")
		Command driveFieldOriented = m_drive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.kTranslationX),
						OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRawAxis(
						OIConstants.kTranslationY), OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(
						m_driverController.getRawAxis(OIConstants.kRotation), OIConstants.kDeadband));

		m_drive.setDefaultCommand(driveFieldOrientedDirectAngle);
		m_elevator.setDefaultCommand(m_elevator.idleCommand());
		m_coralIntake.setDefaultCommand(m_coralIntake.idleCommand());
		m_algaeArm.setDefaultCommand(m_algaeArm.upCommand());
		m_algaeIntake.setDefaultCommand(m_algaeIntake.idleCommand());
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */

	private Command scoreCoralAndReturn(Distance position) {
		return m_elevator.moveToPosition(position)
				.andThen(m_coralIntake.score().raceWith(m_elevator.holdPosition()))
				.andThen(new InstantCommand(m_coralIntake::stop, m_coralIntake))
				.andThen(m_elevator.moveToPosition(ElevatorConstants.Positions.kMinPosition));
	}

	@SuppressWarnings("unused")
	private Command launchL4() {
		return new RunCommand(() -> {
			m_elevator.setVoltage(ElevatorConstants.Outputs.kUp);
		}, m_elevator).alongWith(new RunCommand(() -> {
			if (m_elevator.getPosition().gte(ElevatorConstants.Positions.kL4Launch)) {
				m_coralIntake.outCommand();
			} else {
				m_coralIntake.stop();
			}
		}, m_coralIntake))
				.until(() -> !m_coralIntake.hasCoral()
						&& m_elevator.getPosition().gte(ElevatorConstants.Positions.kMaxPosition))
				.andThen(new InstantCommand(m_coralIntake::stop, m_coralIntake))
				.andThen(m_elevator.moveToPosition(ElevatorConstants.Positions.kMinPosition));
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
		m_operatorsStick.button(OIConstants.kScoreL2)
				.whileTrue(scoreCoralAndReturn(ElevatorConstants.Positions.kL2));
		m_operatorsStick.button(
				OIConstants.kScoreL3).whileTrue(scoreCoralAndReturn(ElevatorConstants.Positions.kL3));
		// m_operatorsStick.button(OIConstants.kScoreL4)
		// .whileTrue(launchL4());
		m_operatorsStick.button(
				OIConstants.kIntakeCoral)
				.whileTrue(m_coralIntake.intake());
		m_operatorsStick.button(OIConstants.kScoreAlgae).whileTrue(m_algaeIntake.outCommand());
		m_operatorsStick.button(OIConstants.kIntakeAlgae).whileTrue(pickupAlgae());
		m_operatorsStick.button(OIConstants.kEjectCoral).whileTrue(m_coralIntake.outCommand());
		m_driverController.button(OIConstants.kZeroGyro).onTrue(new InstantCommand(m_drive::zeroGyro));
		m_driverController.button(OIConstants.kIntakeLeft)
				.whileTrue(lockToHeading(new Rotation2d(DriveConstants.Positions.kLeftIntakeHeading)));
		m_driverController.button(OIConstants.kIntakeRight)
				.whileTrue(lockToHeading(new Rotation2d(DriveConstants.Positions.kRightIntakeHeading)));
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
