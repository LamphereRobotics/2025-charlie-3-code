// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.Units;

public class AlgaeArm extends SubsystemBase {
  private final SparkMax motor = new SparkMax(AlgaeConstants.ArmMotor.kCanId,
      AlgaeConstants.ArmMotor.kMotorType);
  private final RelativeEncoder encoder = motor.getEncoder();

  private final ArmFeedforward feedForward = new ArmFeedforward(
      AlgaeConstants.Feedforward.kS.in(Units.kVoltageUnit),
      AlgaeConstants.Feedforward.kG.in(Units.kVoltageUnit),
      AlgaeConstants.Feedforward.kV);

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      AlgaeConstants.Constraints.kVelocity.in(Units.kAngularVelocityUnit),
      AlgaeConstants.Constraints.kAcceleration.in(Units.kAngularAccelerationUnit));
  private final ProfiledPIDController controller = new ProfiledPIDController(
      AlgaeConstants.PID.kP.in(Units.kVoltageUnit.per(Units.kAngleUnit)),
      AlgaeConstants.PID.kI,
      AlgaeConstants.PID.kD,
      constraints);
  // #endregion

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

    controller.setTolerance(AlgaeConstants.PID.kPositionTolerance.in(
        Units.kAngleUnit),
        AlgaeConstants.PID.kVelocityTolerance.in(Units.kAngularVelocityUnit));
    controller.setIZone(AlgaeConstants.PID.kIZone.in(Units.kAngleUnit));
    controller.setIntegratorRange(-AlgaeConstants.PID.kIntegratorRange.in(Units.kVoltageUnit),
        AlgaeConstants.PID.kIntegratorRange
            .in(Units.kVoltageUnit));
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

  public void setVoltage(Voltage output) {
    motor.setVoltage(output);
  }

  public void stop() {
    setVoltage(Units.kVoltageUnit.zero());
  }
}
