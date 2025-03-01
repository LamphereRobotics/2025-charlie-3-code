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

  public boolean noAlgae() {
    return !hasAlgae();
  }

  public Command inCommand() {
    return setVoltageCommand(AlgaeConstants.Outputs.kIntakeIn).until(this::hasAlgae);
  }

  public Command outCommand() {
    return setVoltageCommand(AlgaeConstants.Outputs.kIntakeOut);
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
