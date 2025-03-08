// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.Units;

public class AlgaeArm extends SubsystemBase {
  private final SparkMax motor = new SparkMax(AlgaeConstants.ArmMotor.kCanId,
      AlgaeConstants.ArmMotor.kMotorType);
  private final RelativeEncoder encoder = motor.getEncoder();

  public AlgaeArm() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
        .inverted(AlgaeConstants.ArmMotor.kInverted)
        .idleMode(AlgaeConstants.ArmMotor.kIdleMode)
        .openLoopRampRate(AlgaeConstants.Constraints.kRampRate);

    motorConfig.encoder
        .positionConversionFactor(AlgaeConstants.Encoder.kPositionConversion.in(Units.kAngleUnit))
        .velocityConversionFactor(
            AlgaeConstants.Encoder.kVelocityConversion.in(Units.kAngularVelocityUnit));

    motorConfig.softLimit
        .forwardSoftLimitEnabled(AlgaeConstants.Positions.kForwardSoftLimitEnabled)
        .forwardSoftLimit(AlgaeConstants.Positions.kMaxPosition.in(Units.kAngleUnit))
        .reverseSoftLimitEnabled(AlgaeConstants.Positions.kReverseSoftLimitEnabled)
        .reverseSoftLimit(AlgaeConstants.Positions.kMinPosition.in(Units.kAngleUnit));

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(AlgaeConstants.Positions.kStartPosition.in(Units.kAngleUnit));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae/Arm/position", encoder.getPosition());
    SmartDashboard.putNumber("Algae/Arm/velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Algae/Arm/voltage", motor.getAppliedOutput() * motor.getBusVoltage());
  }

  public Angle getPosition() {
    return Units.kAngleUnit.of(encoder.getPosition());
  }

  public AngularVelocity getVelocity() {
    return Units.kAngularVelocityUnit.of(encoder.getVelocity());
  }

  public Command upCommand() {
    return run(this::up);
        // .until(() -> this.getPosition().gte(AlgaeConstants.Positions.kHold))
        // .andThen(this.stopCommand());
  }

  public Command downCommand() {
    return run(this::down);
        // .until(() -> this.getPosition().lte(AlgaeConstants.Positions.kPickup))
        // .andThen(this.stopCommand());
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public void up() {
    this.setVoltage(AlgaeConstants.Outputs.kArmUp);
  }

  public void down() {
    this.setVoltage(AlgaeConstants.Outputs.kArmDown);
  }

  public void stop() {
    setVoltage(Units.kVoltageUnit.zero());
  }

  public void setVoltage(Voltage output) {
    motor.setVoltage(output);
  }
}
