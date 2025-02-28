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
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.Units;

public class AlgaeIntake extends SubsystemBase {
  private final SparkMax motor = new SparkMax(AlgaeConstants.IntakeMotor.kCanId,
      AlgaeConstants.IntakeMotor.kMotorType);
  private final DigitalInput limitSwitch = new DigitalInput(AlgaeConstants.LimitSwitch.kPort);

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Algae/Intake/hasAlgae", this.hasAlgae());
    SmartDashboard.putNumber("Algae/Intake/voltage", motor.getAppliedOutput() * motor.getBusVoltage());
  }

  public boolean hasAlgae() {
    return limitSwitch.get();
  }

  public Command inCommand() {
    return run(() -> {
      output(AlgaeConstants.Outputs.kIntakeIn);
    }).until(this::hasAlgae);
  }

  public Command outCommand() {
    return run(() -> {
      output(AlgaeConstants.Outputs.kIntakeOut);
    }).until(() -> {
      return !this.hasAlgae();
    });
  }

  public void output(Voltage output) {
    motor.setVoltage(output);
  }

  public void stop() {
    motor.setVoltage(Units.kVoltageUnit.zero());
  }
}
