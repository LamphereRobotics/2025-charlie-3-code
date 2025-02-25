
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class Config {
  public static final class LeaderMotor {
    public static final int kCanId = 9;
    public static final boolean kInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final MotorType kMotorType = MotorType.kBrushless;
  }

  public static final class FollowerMotor {
    public static final int kCanId = 10;
    public static final boolean kInverted = true;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final MotorType kMotorType = MotorType.kBrushless;
  }

  public static final class Encoder {
    public static final boolean kInverted = false;
    public static final Distance kPositionConversion = Inches.of(1.077);
    public static final LinearVelocity kVelocityConversion = InchesPerSecond.of(0.018);
  }

  public static final class Feedforward {
    public static final Voltage kS = Volts.of(0.12);
    public static final Voltage kG = Volts.of(0.3);
    public static final double kV = 0.117;
  }

  public static final class PID {
    // Coefficients
    // Unit is volts / inch
    public static final Per<VoltageUnit, DistanceUnit> kP = Volts.per(Inch).ofNative(0.133);
    // TODO: Convert to unit type
    // Unit is volts / (inch * second)
    public static final double kI = 0;
    // TODO: Convert to unit type
    // Unit is volts / inch / second
    public static final double kD = 0;

    // Extra config
    public static final Distance kPositionTolerance = Inches.of(0);
    public static final LinearVelocity kVelocityTolerance = InchesPerSecond.of(0);
    public static final Distance kIZone = Inches.of(0);
    public static final Voltage kIntegratorRange = Volts.of(0);
  }

  public static final class Constraints {
    public static final LinearVelocity kVelocity = InchesPerSecond.of(48);
    public static final LinearAcceleration kAcceleration = InchesPerSecond.per(Second).of(122);
  }

  /**
   * All positions will be distances from the ground to the [center of the
   * elevator? end of the outtake?]
   */
  public static final class Positions {
    public static final Distance kL2 = Feet.of(2).plus(Inches.of(7.875));
    public static final Distance kL3 = Feet.of(3).plus(Inches.of(11.625));
    public static final Distance kL4 = Feet.of(6);

    public static final Distance kMinPosition = Inches.of(25.625);
    public static final Distance kMaxPosition = Inches.of(71.5);
    public static final Distance kStartPosition = kMinPosition;
  }

  public static final class Outputs {
    public static final Voltage kVoltage = Volts.of(0.6);
  }

  public static final TimeUnit kTimeUnit = Seconds;
  public static final VoltageUnit kVoltageUnit = Volts;
  public static final DistanceUnit kDistanceUnit = Inches;
  public static final LinearVelocityUnit kLinearVelocityUnit = kDistanceUnit.per(kTimeUnit);
  public static final LinearAccelerationUnit kLinearAccelerationUnit = kLinearVelocityUnit.per(kTimeUnit);
}

public class Elevator extends SubsystemBase {

  private final SparkMax leaderMotor = new SparkMax(Config.LeaderMotor.kCanId,
      Config.LeaderMotor.kMotorType);
  private final SparkMax followerMotor = new SparkMax(Config.FollowerMotor.kCanId,
      Config.FollowerMotor.kMotorType);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();

  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(Config.Feedforward.kS.in(Config.kVoltageUnit),
      Config.Feedforward.kG.in(
          Config.kVoltageUnit),
      Config.Feedforward.kV);

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      Config.Constraints.kVelocity.in(Config.kLinearVelocityUnit),
      Config.Constraints.kAcceleration.in(Config.kLinearAccelerationUnit));
  private final ProfiledPIDController controller = new ProfiledPIDController(
      Config.PID.kP.in(Config.kVoltageUnit.per(Config.kDistanceUnit)),
      Config.PID.kI,
      Config.PID.kD,
      constraints);
  // #endregion

  public Elevator() {
    SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();
    SparkMaxConfig followerMotorConfig = new SparkMaxConfig();

    leaderMotorConfig
        .inverted(Config.LeaderMotor.kInverted)
        .idleMode(Config.LeaderMotor.kIdleMode);

    leaderMotorConfig.encoder
        .positionConversionFactor(Config.Encoder.kPositionConversion.in(Config.kDistanceUnit))
        .velocityConversionFactor(Config.Encoder.kVelocityConversion.in(Config.kLinearVelocityUnit));

    followerMotorConfig
        .follow(leaderMotor, Config.FollowerMotor.kInverted)
        .idleMode(Config.FollowerMotor.kIdleMode);

    leaderMotor.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(Config.Positions.kStartPosition.in(Config.kDistanceUnit));

    controller.setTolerance(Config.PID.kPositionTolerance.in(
        Config.kDistanceUnit),
        Config.PID.kVelocityTolerance.in(Config.kLinearVelocityUnit));
    controller.setIZone(Config.PID.kIZone.in(Config.kDistanceUnit));
    controller.setIntegratorRange(-Config.PID.kIntegratorRange.in(Config.kVoltageUnit), Config.PID.kIntegratorRange
        .in(Config.kVoltageUnit));
  }

  @Override
  public void periodic() {
  }

  public Distance getPosition() {
    return Config.kDistanceUnit.of(encoder.getPosition());
  }

  public LinearVelocity getVelocity() {
    return Config.kLinearVelocityUnit.of(encoder.getVelocity());
  }

  public void setGoal(Distance position) {
    controller.setGoal(position.in(Config.kDistanceUnit));
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  private void usePID() {
    final double pidVoltage = controller.calculate(getPosition().in(Config.kDistanceUnit));
    final double feedForwardVoltage = feedForward.calculate(controller.getSetpoint().velocity);
    this.move(Config.kVoltageUnit.of(pidVoltage + feedForwardVoltage));
  }

  public Command moveToPosition(Distance position) {
    return startRun(() -> {
      setGoal(position);
    }, this::usePID).until(this::atGoal);

  }

  public void move(Voltage output) {
    leaderMotor.setVoltage(output);
  }

  public void moveUp() {
    move(Config.Outputs.kVoltage);
  }

  public void moveDown() {
    move(Config.Outputs.kVoltage.times(-1));
  }

  public void stop() {
    move(Config.kVoltageUnit.zero());
  }
}