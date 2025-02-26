// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.AlgeePickupSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final ClimberSubsystem m_climber = new ClimberSubsystem();
	private final Elevator m_elevator = new Elevator();
	private final Intake m_intake = new Intake();
	private final AlgeePickupSubsystem m_algeePickupSubsystem = new AlgeePickupSubsystem();

	// The driver's controller
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandJoystick m_operatorsStick = new CommandJoystick(OIConstants.kOperatorStickPort);

	private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_robotDrive.zeroHeading();

		m_autonomousChooser.addOption("do nothing", new InstantCommand());
		SmartDashboard.putData(m_autonomousChooser);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		m_robotDrive.setDefaultCommand(m_robotDrive.driveTeleop(m_driverController));
		m_elevator.setDefaultCommand(new RunCommand(m_elevator::stop, m_elevator));
		m_intake.setDefaultCommand(new RunCommand(m_intake::stop, m_intake));
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
				.andThen(m_intake.out().raceWith(m_elevator.holdPosition()))
				.andThen(new InstantCommand(m_intake::stop, m_intake))
				.andThen(m_elevator.moveToPosition(Constants.ElevatorConstants.Positions.kMinPosition));
	}

	private void configureButtonBindings() {
		m_operatorsStick.button(9).whileTrue(scoreCoralAndReturn(Constants.ElevatorConstants.Positions.kL2));
		m_operatorsStick.button(7).whileTrue(scoreCoralAndReturn(Constants.ElevatorConstants.Positions.kL3));
		// m_operatorsStick.button(8)
		// .whileTrue(scoreCoralAndReturn(Constants.ElevatorConstants.Positions.kMaxPosition));
		m_operatorsStick.button(10)
				.whileTrue(m_elevator.moveToPosition(Constants.ElevatorConstants.Positions.kMinPosition)
						.andThen(m_intake.in()));
		m_driverController.leftTrigger().onTrue(m_robotDrive.setSlowModeCommand(true))
				.onFalse(m_robotDrive.setSlowModeCommand(false));
		m_driverController.rightBumper().onTrue(m_robotDrive.setFieldRelativeCommand(false))
				.onFalse(m_robotDrive.setFieldRelativeCommand(true));
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
