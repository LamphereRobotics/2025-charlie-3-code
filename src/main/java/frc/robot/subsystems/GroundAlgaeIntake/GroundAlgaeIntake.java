// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundAlgaeIntake;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Units;

public class GroundAlgaeIntake extends SubsystemBase {
  private final SparkMax motor = new SparkMax(GroundAlgaeIntakeConstants.Motor.kCanId,
      GroundAlgaeIntakeConstants.Motor.kMotorType);
  private final DigitalInput limitSwitch = new DigitalInput(GroundAlgaeIntakeConstants.LimitSwitch.kPort);

  /** Creates a new AlgaeIntake. */
  public GroundAlgaeIntake() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("GroundAlgae/Intake/hasAlgae", this.hasAlgae());
    SmartDashboard.putNumber("GroundAlgae/Intake/voltage", motor.getAppliedOutput() * motor.getBusVoltage());
  }

  public boolean hasAlgae() {
    return limitSwitch.get();
  }

  public boolean noAlgae() {
    return !hasAlgae();
  }

  public Command inCommand() {
    return setVoltageCommand(GroundAlgaeIntakeConstants.Outputs.kIn).until(this::hasAlgae);
  }

  public Command outCommand() {
    return setVoltageCommand(GroundAlgaeIntakeConstants.Outputs.kOut);
  }

  public Command idleCommand() {
    return run(() -> {
      if (this.hasAlgae()) {
        this.setVoltage(GroundAlgaeIntakeConstants.Outputs.kHold);
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
