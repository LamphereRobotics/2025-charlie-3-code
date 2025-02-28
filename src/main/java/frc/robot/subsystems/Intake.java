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

public class Intake extends SubsystemBase {
  private final SparkMax motor = new SparkMax(CoralIntakeConstants.Motor.kCanId,
      CoralIntakeConstants.Motor.kMotorType);
  private final DigitalInput limitSwitch = new DigitalInput(CoralIntakeConstants.LimitSwitch.kPort);

  public Intake() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("CoralIntake/hasCoral", limitSwitch.get());
    SmartDashboard.putNumber("CoralIntake/outputVoltage", motor.getAppliedOutput() * motor.getBusVoltage());
  }

  public boolean hasCoral() {
    return limitSwitch.get();
  }

  public Command in() {
    return run(() -> {
      this.output(CoralIntakeConstants.Outputs.kIn);
    }).until(this::hasCoral);
  }

  public Command out() {
    return run(() -> {
      this.output(CoralIntakeConstants.Outputs.kOut);
    }).until(() -> {
      return !this.hasCoral();
    });
  }

  public void output(Voltage output) {
    motor.setVoltage(output);
  }

  public void stop() {
    motor.set(0);
  }

}
