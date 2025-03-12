// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.UpperAlgaeArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.Radians;
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

  private final ArmFeedforward feedForward = new ArmFeedforward(
      UpperAlgaeArmConstants.Feedforward.kS.in(Units.kVoltageUnit),
      UpperAlgaeArmConstants.Feedforward.kG.in(
          Units.kVoltageUnit),
      UpperAlgaeArmConstants.Feedforward.kV.in(Units.kAngularKVUnit));

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      UpperAlgaeArmConstants.Constraints.kVelocity.in(
          Units.kAngularVelocityUnit),
      UpperAlgaeArmConstants.Constraints.kAcceleration.in(Units.kAngularAccelerationUnit));
  private final ProfiledPIDController controller = new ProfiledPIDController(
      UpperAlgaeArmConstants.PID.kP.in(
          Units.kVoltageUnit.per(
              Units.kAngleUnit)),
      UpperAlgaeArmConstants.PID.kI,
      UpperAlgaeArmConstants.PID.kD,
      constraints);

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

  public void setGoal(Angle position) {
    controller.setGoal(position.in(Units.kAngleUnit));
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  private void usePID() {
    final double pidVoltage = controller.calculate(getPosition().in(Units.kAngleUnit));
    final double feedForwardVoltage = feedForward.calculate(getPosition().in(Radians),
        controller.getSetpoint().velocity);
    this.setVoltage(Units.kVoltageUnit.of(pidVoltage + feedForwardVoltage));
  }

  public Command holdPosition() {
    return run(this::usePID);
  }

  public Command moveToPosition(Angle position) {
    return startRun(() -> {
      setGoal(position);
    }, this::usePID).until(this::atGoal);
  }

  public Command idleCommand() {
    return run(this::hold);
  }

  public void hold() {
    final double feedForwardVoltage = feedForward.calculate(getPosition().in(Radians), 0);
    this.setVoltage(Units.kVoltageUnit.of(feedForwardVoltage));
  }

  public void stop() {
    setVoltage(Units.kVoltageUnit.zero());
  }

  public void setVoltage(Voltage output) {
    motor.setVoltage(output);
  }
}
