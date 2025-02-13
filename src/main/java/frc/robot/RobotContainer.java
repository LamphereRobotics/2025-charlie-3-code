// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outake;
import frc.robot.subsystems.AlgaePickup;
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
	private final Drive robotDrive = new Drive();
	private final Climber climber = new Climber();
	private final Elevator elevator = new Elevator();
	private final Intake intake = new Intake();
	private final Outake outake = new Outake();
	private final AlgaePickup algaePickupSubsystem = new AlgaePickup();

	// The driver's controller
	private final CommandXboxController driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandJoystick operatorsStick = new CommandJoystick(OIConstants.kOperatorStickPort);

	private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		robotDrive.zeroHeading();

		m_autonomousChooser.addOption("do nothing", new InstantCommand());
		SmartDashboard.putData(m_autonomousChooser);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		robotDrive.setDefaultCommand(robotDrive.driveTeleop(driverController));
		elevator.setDefaultCommand(new RunCommand(elevator::stop, elevator));
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

	private void configureButtonBindings() {

		operatorsStick.button(1).whileTrue(new RunCommand(elevator::moveUp, elevator));
		operatorsStick.button(2).whileTrue(new RunCommand(elevator::moveDown, elevator));
		driverController.a().onTrue(robotDrive.resetGyro());
		driverController.leftTrigger().onTrue(robotDrive.setSlowModeCommand(true))
				.onFalse(robotDrive.setSlowModeCommand(false));
		driverController.rightBumper().onTrue(robotDrive.setFieldRelativeCommand(false))
				.onFalse(robotDrive.setFieldRelativeCommand(true));
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
