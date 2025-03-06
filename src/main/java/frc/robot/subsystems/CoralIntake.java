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
  private final DigitalInput innerLimitSwitch = new DigitalInput(CoralIntakeConstants.LimitSwitch.kInnerPort);
  private final DigitalInput outerLimitSwitch = new DigitalInput(CoralIntakeConstants.LimitSwitch.kOuterPort);

  public CoralIntake() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("CoralIntake/hasCoral", hasInnerCoral());
    SmartDashboard.putNumber("CoralIntake/outputVoltage", motor.getAppliedOutput() * motor.getBusVoltage());
  }

  public boolean hasInnerCoral() {
    return innerLimitSwitch.get();
  }

  public boolean noInnerCoral() {
    return !hasInnerCoral();
  }

  public boolean hasOuterCoral() {
    return outerLimitSwitch.get();
  }

  public boolean noOuterCoral() {
    return !hasOuterCoral();
  }

  public boolean hasAnyCoral() {
    return hasInnerCoral() || hasOuterCoral();
  }

  public boolean noAnyCoral() {
    return !hasAnyCoral();
  }

  public Command intake() {
    return this.inCommand().until(this::hasInnerCoral)
        .andThen(this.in2Command().until(this::hasOuterCoral));
  }

  public Command score() {
    return this.outCommand().until(this::noAnyCoral);
  }

  public Command inCommand() {
    return this.setVoltageCommand(CoralIntakeConstants.Outputs.kIn);
  }

  public Command in2Command() {
    return this.setVoltageCommand(CoralIntakeConstants.Outputs.kIn2);
  }

  public Command outCommand() {
    return this.setVoltageCommand(CoralIntakeConstants.Outputs.kOut);
  }

  public Command idleCommand() {
    return run(() -> {
      if (this.hasOuterCoral()) {
        this.setVoltage(CoralIntakeConstants.Outputs.kHold);
      } else {
        this.stop();
      }
    });
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
