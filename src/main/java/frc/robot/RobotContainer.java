// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.AlgaeArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AlgaeIntake;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	// private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final Drive m_drive = new Drive(new File(Filesystem.getDeployDirectory(),
			"swerve"), Pose2d.kZero);
	private final Elevator m_elevator = new Elevator();
	private final CoralIntake m_coralIntake = new CoralIntake();
	private final AlgaeArm m_algaeArm = new AlgaeArm();
	private final AlgaeIntake m_algaeIntake = new AlgaeIntake();

	// The driver's controller
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandJoystick m_operatorsStick = new CommandJoystick(OIConstants.kOperatorStickPort);

	private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_autonomousChooser.addOption("do nothing", new InstantCommand());
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
		m_elevator.setDefaultCommand(new RunCommand(m_elevator::stop, m_elevator));
		m_coralIntake.setDefaultCommand(new RunCommand(m_coralIntake::stop, m_coralIntake));
		m_algaeArm.setDefaultCommand(new RunCommand(() -> {
			double input = -MathUtil.applyDeadband(m_operatorsStick.getRawAxis(1), OIConstants.kDeadband);
			m_algaeArm.setVoltage(AlgaeConstants.Outputs.kArmMax.times(input));
		}, m_algaeArm));
		m_algaeIntake.setDefaultCommand(new RunCommand(m_algaeIntake::stop, m_algaeIntake));
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
				.andThen(m_elevator.moveToPosition(Constants.ElevatorConstants.Positions.kMinPosition));
	}

	@SuppressWarnings("unused")
	private Command launchL4() {
		return new RunCommand(() -> {
			m_elevator.move(Constants.ElevatorConstants.Outputs.kUp);
		}, m_elevator).alongWith(new RunCommand(() -> {
			if (m_elevator.getPosition().gte(Constants.ElevatorConstants.Positions.kL4Launch)) {
				m_coralIntake.outCommand();
			} else {
				m_coralIntake.stop();
			}
		}, m_coralIntake))
				.until(() -> !m_coralIntake.hasCoral()
						&& m_elevator.getPosition().gte(Constants.ElevatorConstants.Positions.kMaxPosition))
				.andThen(new InstantCommand(m_coralIntake::stop, m_coralIntake))
				.andThen(m_elevator.moveToPosition(Constants.ElevatorConstants.Positions.kMinPosition));
	}

	private void configureButtonBindings() {
		m_operatorsStick.button(OIConstants.kScoreL2)
				.whileTrue(scoreCoralAndReturn(Constants.ElevatorConstants.Positions.kL2));
		m_operatorsStick.button(
				OIConstants.kScoreL3).whileTrue(scoreCoralAndReturn(Constants.ElevatorConstants.Positions.kL3));
		// m_operatorsStick.button(OIConstants.kScoreL4)
		// .whileTrue(launchL4());
		m_operatorsStick.button(
				OIConstants.kIntakeCoral)
				.whileTrue(m_elevator.moveToPosition(Constants.ElevatorConstants.Positions.kMinPosition)
						.andThen(m_coralIntake.intake()));
		m_operatorsStick.button(OIConstants.kScoreAlgae).whileTrue(m_algaeIntake.outCommand());
		m_operatorsStick.button(OIConstants.kIntakeAlgae).whileTrue(m_algaeIntake.inCommand());
		m_driverController.button(OIConstants.kZeroGyro).onTrue(new InstantCommand(m_drive::zeroGyro));
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
