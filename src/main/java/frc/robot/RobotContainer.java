// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.YagslDrive;
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
	private final YagslDrive m_yagslDrive = new YagslDrive(new File(Filesystem.getDeployDirectory(),
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
		// m_robotDrive.zeroHeading();

		m_autonomousChooser.addOption("do nothing", new InstantCommand());
		SmartDashboard.putData(m_autonomousChooser);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// m_robotDrive.setDefaultCommand(m_robotDrive.driveTeleop(m_driverController));
		Command driveFieldOrientedDirectAngle = m_yagslDrive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDeadband),
				() -> -m_driverController.getRightX(),
				() -> -m_driverController.getRightY());
		Command driveFieldOriented = m_yagslDrive.driveCommand(
				() -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDeadband),
				() -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDeadband));

		m_yagslDrive.setDefaultCommand(driveFieldOrientedDirectAngle);
		m_elevator.setDefaultCommand(new RunCommand(m_elevator::stop, m_elevator));
		m_coralIntake.setDefaultCommand(new RunCommand(m_coralIntake::stop, m_coralIntake));
		m_algaeArm.setDefaultCommand(new RunCommand(() -> {
			double output = -MathUtil.applyDeadband(m_operatorsStick.getRawAxis(1), 0.25) * 4;
			m_algaeArm.move(Volts.of(output));
			SmartDashboard.putNumber("Algae/Arm/input", m_operatorsStick.getRawAxis(1));
			SmartDashboard.putNumber("Algae/Arm/output", output);

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

	private Command launchL4() {
		return new RunCommand(() -> {
			m_elevator.move(Constants.ElevatorConstants.Outputs.kUp);
		}, m_elevator).alongWith(new RunCommand(() -> {
			if (m_elevator.getPosition().gte(Constants.ElevatorConstants.Positions.kL4Launch)) {
				m_coralIntake.out();
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
		m_operatorsStick.button(9).whileTrue(scoreCoralAndReturn(Constants.ElevatorConstants.Positions.kL2));
		m_operatorsStick.button(7).whileTrue(scoreCoralAndReturn(Constants.ElevatorConstants.Positions.kL3));
		// m_operatorsStick.button(8)
		// .whileTrue(launchL4());
		m_operatorsStick.button(10)
				.whileTrue(m_elevator.moveToPosition(Constants.ElevatorConstants.Positions.kMinPosition)
						.andThen(m_coralIntake.intake()));
		m_operatorsStick.button(1).whileTrue(m_algaeIntake.outCommand());
		m_operatorsStick.button(4).whileTrue(m_algaeIntake.inCommand());
		// m_operatorsStick.button(5).whileTrue(new RunCommand(() ->
		// m_algaeArm.move(Volts.of(-1)), m_algaeArm));
		// m_operatorsStick.button(6).whileTrue(new RunCommand(() ->
		// m_algaeArm.move(Volts.of(1)), m_algaeArm));
		m_driverController.a().onTrue(new InstantCommand(m_yagslDrive::zeroGyro));
		// m_driverController.leftTrigger().onTrue(m_robotDrive.setSlowModeCommand(true))
		// .onFalse(m_robotDrive.setSlowModeCommand(false));
		// m_driverController.rightBumper().onTrue(m_robotDrive.setFieldRelativeCommand(false))
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
