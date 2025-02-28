
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final SparkMax leaderMotor = new SparkMax(ElevatorConstants.LeaderMotor.kCanId,
      ElevatorConstants.LeaderMotor.kMotorType);
  private final SparkMax followerMotor = new SparkMax(ElevatorConstants.FollowerMotor.kCanId,
      ElevatorConstants.FollowerMotor.kMotorType);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();

  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(
      ElevatorConstants.Feedforward.kS.in(ElevatorConstants.kVoltageUnit),
      ElevatorConstants.Feedforward.kG.in(
          ElevatorConstants.kVoltageUnit),
      ElevatorConstants.Feedforward.kV);

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      ElevatorConstants.Constraints.kVelocity.in(ElevatorConstants.kLinearVelocityUnit),
      ElevatorConstants.Constraints.kAcceleration.in(ElevatorConstants.kLinearAccelerationUnit));
  private final ProfiledPIDController controller = new ProfiledPIDController(
      ElevatorConstants.PID.kP.in(ElevatorConstants.kVoltageUnit.per(ElevatorConstants.kDistanceUnit)),
      ElevatorConstants.PID.kI,
      ElevatorConstants.PID.kD,
      constraints);
  // #endregion

  public Elevator() {
    SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();
    SparkMaxConfig followerMotorConfig = new SparkMaxConfig();

    leaderMotorConfig
        .inverted(ElevatorConstants.LeaderMotor.kInverted)
        .idleMode(ElevatorConstants.LeaderMotor.kIdleMode)
        .openLoopRampRate(ElevatorConstants.Constraints.kRampRate);

    leaderMotorConfig.encoder
        .positionConversionFactor(ElevatorConstants.Encoder.kPositionConversion.in(ElevatorConstants.kDistanceUnit))
        .velocityConversionFactor(
            ElevatorConstants.Encoder.kVelocityConversion.in(ElevatorConstants.kLinearVelocityUnit));

    leaderMotorConfig.softLimit
        .forwardSoftLimitEnabled(ElevatorConstants.Positions.kForwardSoftLimitEnabled)
        .forwardSoftLimit(ElevatorConstants.Positions.kMaxPosition.in(ElevatorConstants.kDistanceUnit))
        .reverseSoftLimitEnabled(ElevatorConstants.Positions.kReverseSoftLimitEnabled)
        .reverseSoftLimit(ElevatorConstants.Positions.kMinPosition.in(ElevatorConstants.kDistanceUnit));

    followerMotorConfig
        .inverted(ElevatorConstants.FollowerMotor.kInverted)
        .follow(leaderMotor, ElevatorConstants.FollowerMotor.kInverted)
        .idleMode(ElevatorConstants.FollowerMotor.kIdleMode)
        .openLoopRampRate(ElevatorConstants.Constraints.kRampRate);

    followerMotorConfig.encoder
        .positionConversionFactor(ElevatorConstants.Encoder.kPositionConversion.in(ElevatorConstants.kDistanceUnit))
        .velocityConversionFactor(
            ElevatorConstants.Encoder.kVelocityConversion.in(ElevatorConstants.kLinearVelocityUnit));

    followerMotorConfig.softLimit
        .forwardSoftLimitEnabled(ElevatorConstants.Positions.kForwardSoftLimitEnabled)
        .forwardSoftLimit(ElevatorConstants.Positions.kMaxPosition.in(ElevatorConstants.kDistanceUnit))
        .reverseSoftLimitEnabled(ElevatorConstants.Positions.kReverseSoftLimitEnabled)
        .reverseSoftLimit(ElevatorConstants.Positions.kMinPosition.in(ElevatorConstants.kDistanceUnit));

    leaderMotor.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerMotor.getEncoder()
        .setPosition(ElevatorConstants.Positions.kStartPosition.in(ElevatorConstants.kDistanceUnit));
    encoder.setPosition(ElevatorConstants.Positions.kStartPosition.in(ElevatorConstants.kDistanceUnit));

    controller.setTolerance(ElevatorConstants.PID.kPositionTolerance.in(
        ElevatorConstants.kDistanceUnit),
        ElevatorConstants.PID.kVelocityTolerance.in(ElevatorConstants.kLinearVelocityUnit));
    controller.setIZone(ElevatorConstants.PID.kIZone.in(ElevatorConstants.kDistanceUnit));
    controller.setIntegratorRange(-ElevatorConstants.PID.kIntegratorRange.in(ElevatorConstants.kVoltageUnit),
        ElevatorConstants.PID.kIntegratorRange
            .in(ElevatorConstants.kVoltageUnit));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator-leader-encoder-position", encoder.getPosition());
    SmartDashboard.putNumber("elevator-follower-encoder-position", followerMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("elevator-encoder-velocity", encoder.getVelocity());
    SmartDashboard.putNumber("elevator-motor-voltage", leaderMotor.getAppliedOutput() * leaderMotor.getBusVoltage());
  }

  public Distance getPosition() {
    return ElevatorConstants.kDistanceUnit.of(encoder.getPosition());
  }

  public LinearVelocity getVelocity() {
    return ElevatorConstants.kLinearVelocityUnit.of(encoder.getVelocity());
  }

  public void setGoal(Distance position) {
    controller.setGoal(position.in(ElevatorConstants.kDistanceUnit));
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  private void usePID() {
    final double pidVoltage = controller.calculate(getPosition().in(ElevatorConstants.kDistanceUnit));
    final double feedForwardVoltage = feedForward.calculate(controller.getSetpoint().velocity);
    this.move(ElevatorConstants.kVoltageUnit.of(pidVoltage + feedForwardVoltage));
  }

  public Command holdPosition() {
    return run(this::usePID);
  }

  public Command moveToPosition(Distance position) {
    return startRun(() -> {
      setGoal(position);
    }, this::usePID).until(this::atGoal);
  }

  public void move(Voltage output) {
    leaderMotor.setVoltage(output);
  }

  public void stop() {
    move(ElevatorConstants.kVoltageUnit.zero());
  }
}