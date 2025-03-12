// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.UpperAlgaeIntake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Units;

public class UpperAlgaeIntake extends SubsystemBase {
  private final SparkMax leaderMotor = new SparkMax(UpperAlgaeIntakeConstants.LeaderMotor.kCanId,
      UpperAlgaeIntakeConstants.LeaderMotor.kMotorType);
  private final SparkMax followerMotor = new SparkMax(UpperAlgaeIntakeConstants.LeaderMotor.kCanId,
      UpperAlgaeIntakeConstants.LeaderMotor.kMotorType);
  private final DigitalInput limitSwitch = new DigitalInput(UpperAlgaeIntakeConstants.LimitSwitch.kPort);

  public UpperAlgaeIntake() {
    SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();
    SparkMaxConfig followerMotorConfig = new SparkMaxConfig();

    leaderMotorConfig
      .inverted(UpperAlgaeIntakeConstants.LeaderMotor.kInverted)
      .idleMode(UpperAlgaeIntakeConstants.LeaderMotor.kIdleMode);

    followerMotorConfig
        .inverted(UpperAlgaeIntakeConstants.FollowerMotor.kInverted)
        .idleMode(UpperAlgaeIntakeConstants.FollowerMotor.kIdleMode)
        .follow(leaderMotor, UpperAlgaeIntakeConstants.FollowerMotor.kInverted);

    leaderMotor.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("UpperAlgae/Intake/hasAlgae", this.hasAlgae());
    SmartDashboard.putNumber("UpperAlgae/Intake/leader/voltage",
        leaderMotor.getAppliedOutput() * leaderMotor.getBusVoltage());
    SmartDashboard.putNumber("UpperAlgae/Intake/follower/voltage",
        followerMotor.getAppliedOutput() * followerMotor.getBusVoltage());
  }

  public boolean hasAlgae() {
    return limitSwitch.get();
  }

  public boolean noAlgae() {
    return !hasAlgae();
  }

  public Command inCommand() {
    return setVoltageCommand(UpperAlgaeIntakeConstants.Outputs.kIn).until(this::hasAlgae);
  }

  public Command outCommand() {
    return setVoltageCommand(UpperAlgaeIntakeConstants.Outputs.kOut);
  }

  public Command idleCommand() {
    return run(() -> {
      if (this.hasAlgae()) {
        this.setVoltage(UpperAlgaeIntakeConstants.Outputs.kHold);
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
    leaderMotor.setVoltage(output);
  }

}
