// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final SparkMax m_spitMotor = new SparkMax(13,
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final DigitalInput IntakeSensor = new DigitalInput(2);

  public Intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Sensor", IntakeSensor.get());
  }

  public void in() {
    if (!IntakeSensor.get()) {
      m_spitMotor.set(0.15);
    } else {
      stop();
    }
  }

  public void out() {
    m_spitMotor.set(0.15);
  }

  public void stop() {
    // call move with a - value
    m_spitMotor.set(0);
  }

}
