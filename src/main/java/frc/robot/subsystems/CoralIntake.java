// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.Units;

public class CoralIntake extends SubsystemBase {
  private final SparkMax motor = new SparkMax(CoralIntakeConstants.Motor.kCanId,
      CoralIntakeConstants.Motor.kMotorType);
  private final DigitalInput limitSwitch = new DigitalInput(CoralIntakeConstants.LimitSwitch.kPort);

  public CoralIntake() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("CoralIntake/hasCoral", hasCoral());
    SmartDashboard.putNumber("CoralIntake/outputVoltage", motor.getAppliedOutput() * motor.getBusVoltage());
  }

  public boolean hasCoral() {
    return limitSwitch.get();
  }

  public boolean noCoral() {
    return !hasCoral();
  }

  public Command intake() {
    return this.inCommand().until(this::hasCoral);
  }

  public Command score() {
    return this.outCommand().until(this::noCoral);
  }

  public Command inCommand() {
    return this.setVoltageCommand(CoralIntakeConstants.Outputs.kIn);
  }

  public Command outCommand() {
    return this.setVoltageCommand(CoralIntakeConstants.Outputs.kOut);
  }

  public Command setVoltageCommand(Voltage output) {
    return run(() -> this.setVoltage(output));
  }

  public void stop() {
    this.setVoltage(Units.kVoltageUnit.zero());
  }

  public void setVoltage(Voltage output) {
    motor.setVoltage(output);
  }
}
