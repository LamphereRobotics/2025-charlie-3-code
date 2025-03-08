// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.UpperAlgaeArm;

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
import frc.robot.Constants.Units;

public class UpperAlgaeArm extends SubsystemBase {
  private final SparkMax motor = new SparkMax(UpperAlgaeArmConstants.ArmMotor.kCanId,
      UpperAlgaeArmConstants.ArmMotor.kMotorType);
  private final RelativeEncoder encoder = motor.getEncoder();

  public UpperAlgaeArm() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
        .inverted(UpperAlgaeArmConstants.ArmMotor.kInverted)
        .idleMode(UpperAlgaeArmConstants.ArmMotor.kIdleMode)
        .openLoopRampRate(UpperAlgaeArmConstants.Outputs.kRampRate);

    motorConfig.encoder
        .positionConversionFactor(UpperAlgaeArmConstants.Encoder.kPositionConversion.in(Units.kAngleUnit))
        .velocityConversionFactor(
            UpperAlgaeArmConstants.Encoder.kVelocityConversion.in(Units.kAngularVelocityUnit));

    motorConfig.softLimit
        .forwardSoftLimitEnabled(UpperAlgaeArmConstants.Positions.kForwardSoftLimitEnabled)
        .forwardSoftLimit(UpperAlgaeArmConstants.Positions.kMaxPosition.in(Units.kAngleUnit))
        .reverseSoftLimitEnabled(UpperAlgaeArmConstants.Positions.kReverseSoftLimitEnabled)
        .reverseSoftLimit(UpperAlgaeArmConstants.Positions.kMinPosition.in(Units.kAngleUnit));

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(UpperAlgaeArmConstants.Positions.kStartPosition.in(Units.kAngleUnit));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("UpperAlgae/Arm/position", encoder.getPosition());
    SmartDashboard.putNumber("UpperAlgae/Arm/velocity", encoder.getVelocity());
    SmartDashboard.putNumber("UpperAlgae/Arm/voltage", motor.getAppliedOutput() * motor.getBusVoltage());
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
    this.setVoltage(UpperAlgaeArmConstants.Outputs.kArmUp);
  }

  public void down() {
    this.setVoltage(UpperAlgaeArmConstants.Outputs.kArmDown);
  }

  public void stop() {
    setVoltage(Units.kVoltageUnit.zero());
  }

  public void setVoltage(Voltage output) {
    motor.setVoltage(output);
  }
}
