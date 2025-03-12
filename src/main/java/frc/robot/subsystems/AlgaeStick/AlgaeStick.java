// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeStick;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Units;

public class AlgaeStick extends SubsystemBase {
  private final SparkMax motor = new SparkMax(AlgaeStickConstants.Motor.kCanId,
      AlgaeStickConstants.Motor.kMotorType);
  private final RelativeEncoder encoder = motor.getEncoder();

  private final PIDController pidController = new PIDController(
      AlgaeStickConstants.PID.kP,
      AlgaeStickConstants.PID.kI,
      AlgaeStickConstants.PID.kD);

  public AlgaeStick() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
        .inverted(AlgaeStickConstants.Motor.kInverted)
        .idleMode(AlgaeStickConstants.Motor.kIdleMode)
        .openLoopRampRate(AlgaeStickConstants.Outputs.kRampRate);

    motorConfig.encoder
        .positionConversionFactor(AlgaeStickConstants.Encoder.kPositionConversion.in(Units.kAngleUnit))
        .velocityConversionFactor(
            AlgaeStickConstants.Encoder.kVelocityConversion.in(Units.kAngularVelocityUnit));

    motorConfig.softLimit
        .forwardSoftLimitEnabled(AlgaeStickConstants.Positions.kForwardSoftLimitEnabled)
        .forwardSoftLimit(AlgaeStickConstants.Positions.kMaxPosition.in(Units.kAngleUnit))
        .reverseSoftLimitEnabled(AlgaeStickConstants.Positions.kReverseSoftLimitEnabled)
        .reverseSoftLimit(AlgaeStickConstants.Positions.kMinPosition.in(Units.kAngleUnit));

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(AlgaeStickConstants.Positions.kStartPosition.in(Units.kAngleUnit));

    pidController.setTolerance(
        AlgaeStickConstants.PID.kPositionTolerance.in(Units.kAngleUnit),
        AlgaeStickConstants.PID.kVelocityTolerance.in(Units.kAngularVelocityUnit));
    pidController.setIZone(AlgaeStickConstants.PID.kIZone.in(Units.kAngleUnit));
    pidController.setIntegratorRange(
        -AlgaeStickConstants.PID.kIntegratorRange.in(Units.kVoltageUnit),
        AlgaeStickConstants.PID.kIntegratorRange.in(Units.kVoltageUnit));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AlgaeStick/position", encoder.getPosition());
    SmartDashboard.putNumber("AlgaeStick/velocity", encoder.getVelocity());
    SmartDashboard.putNumber("AlgaeStick/voltage", motor.getAppliedOutput() * motor.getBusVoltage());
    SmartDashboard.putNumber("AlgaeStick/setpoint", pidController.getSetpoint());
    SmartDashboard.putNumber("AlgaeStick/pidOutput", pidController.calculate(this.getPosition().in(Units.kAngleUnit)));
  }

  public Angle getPosition() {
    return Units.kAngleUnit.of(encoder.getPosition());
  }

  public AngularVelocity getVelocity() {
    return Units.kAngularVelocityUnit.of(encoder.getVelocity());
  }

  public boolean atSetpoint() {
    return this.pidController.atSetpoint();
  }

  public void setSetpoint(Angle setpoint) {
    pidController.setSetpoint(setpoint.in(Units.kAngleUnit));
  }

  public void usePid() {
    this.setVoltage(Units.kVoltageUnit.of(pidController.calculate(this.getPosition().in(Units.kAngleUnit))));
  }

  public Command highCommand() {
    return this.holdPositionCommand(AlgaeStickConstants.Positions.kHigh);

  }

  public Command lowCommand() {
    return this.holdPositionCommand(AlgaeStickConstants.Positions.kLow);
  }

  public Command holdPositionCommand(Angle targetAngle) {
    return startRun(() -> this.setSetpoint(targetAngle), this::usePid);
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public void up() {
    this.setVoltage(AlgaeStickConstants.Outputs.kUp);
  }

  public void down() {
    this.setVoltage(AlgaeStickConstants.Outputs.kDown);
  }

  public void stop() {
    setVoltage(Units.kVoltageUnit.zero());
  }

  public void setVoltage(Voltage output) {
    motor.setVoltage(output);
  }
}
