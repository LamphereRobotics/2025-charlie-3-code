// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.2;

    public static final class Positions {
      public static final Angle kLeftIntakeHeading = Degrees.of(306);
      public static final Angle kRightIntakeHeading = Degrees.of(54);
    }
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorStickPort = 1;

    public static final double kDeadband = 0.15;

    // Driver axes
    public static final int kTranslationX = XboxController.Axis.kLeftY.value;
    public static final int kTranslationY = XboxController.Axis.kLeftX.value;
    public static final int kHeadingX = XboxController.Axis.kRightX.value;
    public static final int kHeadingY = XboxController.Axis.kRightY.value;
    public static final int kRotation = XboxController.Axis.kRightX.value;

    // Driver buttons
    public static final int kZeroGyro = XboxController.Button.kA.value;
    public static final int kSlowMode = XboxController.Axis.kLeftTrigger.value;
    public static final int kRobotRelative = XboxController.Button.kRightBumper.value;
    public static final int kIntakeLeft = XboxController.Button.kLeftBumper.value;
    public static final int kIntakeRight = XboxController.Button.kRightBumper.value;

    // Operator axes
    public static final int kMoveAlgaeArm = 1;

    // Operator buttons
    public static final int kIntakeCoral = 10;
    public static final int kScoreL2 = 9;
    public static final int kScoreL3 = 7;
    public static final int kScoreL4 = 8;
    public static final int kIntakeAlgae = 4;
    public static final int kScoreAlgae = 1;
  }

  public static final class ElevatorConstants {
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
      public static final Per<VoltageUnit, DistanceUnit> kP = Volts.per(Inch).ofNative(0.3);
      // TODO: Convert to unit type
      // Unit is volts / (inch * second)
      public static final double kI = 0;
      // TODO: Convert to unit type
      // Unit is volts / inch / second
      public static final double kD = 0.03;

      // Extra config
      public static final Distance kPositionTolerance = Inches.of(1);
      public static final LinearVelocity kVelocityTolerance = InchesPerSecond.of(0.25);
      public static final Distance kIZone = Inches.of(0);
      public static final Voltage kIntegratorRange = Volts.of(0);
    }

    public static final class Constraints {
      public static final LinearVelocity kVelocity = InchesPerSecond.of(48);
      public static final LinearAcceleration kAcceleration = InchesPerSecond.per(Second).of(122);
      public static final double kRampRate = 0.25;
    }

    /**
     * All positions will be distances from the ground to the [center of the
     * elevator? end of the outtake?]
     */
    public static final class Positions {
      public static final Distance kL2 = Feet.of(2).plus(Inches.of(7.875));
      public static final Distance kL3 = Feet.of(3).plus(Inches.of(11.625));
      public static final Distance kL4 = Feet.of(6);
      public static final Distance kL4Launch = Feet.of(5).plus(Inches.of(10));

      public static final Distance kMinPosition = Inches.of(25.625);
      public static final Distance kMaxPosition = Inches.of(71);
      public static final Distance kStartPosition = kMinPosition;

      public static final boolean kForwardSoftLimitEnabled = true;
      public static final boolean kReverseSoftLimitEnabled = true;
    }

    public static final class Outputs {
      public static final Voltage kUp = Volts.of(6);
    }

    public static final TimeUnit kTimeUnit = Seconds;
    public static final VoltageUnit kVoltageUnit = Volts;
    public static final DistanceUnit kDistanceUnit = Inches;
    public static final LinearVelocityUnit kLinearVelocityUnit = kDistanceUnit.per(kTimeUnit);
    public static final LinearAccelerationUnit kLinearAccelerationUnit = kLinearVelocityUnit.per(kTimeUnit);
  }

  public static final class AlgaeConstants {
    public static final class ArmMotor {
      public static final int kCanId = 11;
      public static final boolean kInverted = false;
      public static final IdleMode kIdleMode = IdleMode.kBrake;
      public static final MotorType kMotorType = MotorType.kBrushless;
    }

    public static final class IntakeMotor {
      public static final int kCanId = 12;
      public static final boolean kInverted = true;
      public static final IdleMode kIdleMode = IdleMode.kBrake;
      public static final MotorType kMotorType = MotorType.kBrushless;
    }

    public static final class Encoder {
      public static final Angle kPositionConversion = Degrees.of(1.0);
      public static final AngularVelocity kVelocityConversion = DegreesPerSecond.of(1.0);
    }

    public static final class LimitSwitch {
      public static final int kPort = 9;
    }

    public static final class Feedforward {
      public static final Voltage kS = Volts.of(0.0);
      public static final Voltage kG = Volts.of(0.0);
      public static final double kV = 0.0;
    }

    public static final class PID {
      // Coefficients
      public static final Per<VoltageUnit, AngleUnit> kP = Volts.per(Degrees).ofNative(0.0);
      // TODO: Convert to unit type
      // Unit is volts / (degree * second)
      public static final double kI = 0.0;
      // TODO: Convert to unit type
      // Unit is volts / degree / second
      public static final double kD = 0.0;

      // Extra config
      public static final Angle kPositionTolerance = Degrees.of(5);
      public static final AngularVelocity kVelocityTolerance = DegreesPerSecond.of(2);
      public static final Angle kIZone = Degrees.of(0);
      public static final Voltage kIntegratorRange = Volts.of(0);
    }

    public static final class Constraints {
      public static final AngularVelocity kVelocity = DegreesPerSecond.of(360);
      public static final AngularAcceleration kAcceleration = DegreesPerSecond.per(Second).of(360);
      public static final double kRampRate = 0.25;
    }

    public static final class Positions {
      public static final Angle kPickup = Degrees.of(45);
      public static final Angle kHold = Degrees.of(90);
      public static final Angle kScore = Degrees.of(90);

      public static final Angle kMinPosition = Degrees.of(20);
      public static final Angle kMaxPosition = Degrees.of(90);
      public static final Angle kStartPosition = Degrees.of(110);

      public static final boolean kForwardSoftLimitEnabled = false;
      public static final boolean kReverseSoftLimitEnabled = false;
    }

    public static final class Outputs {
      public static final Voltage kArmMax = Volts.of(4);
      public static final Voltage kIntakeIn = Volts.of(-6);
      public static final Voltage kIntakeOut = Volts.of(12);
      public static final Voltage kHold = Volts.of(-2);
    }
  }

  public static final class CoralIntakeConstants {
    public static final class Motor {
      public static final int kCanId = 13;
      public static final boolean kInverted = false;
      public static final IdleMode kIdleMode = IdleMode.kBrake;
      public static final MotorType kMotorType = MotorType.kBrushless;
    }

    public static final class LimitSwitch {
      public static final int kPort = 2;
    }

    public static final class Outputs {
      public static final Voltage kOut = Volts.of(12);
      public static final Voltage kIn = Volts.of(2);
      public static final Voltage kHold = Volts.of(-1);
    }
  }

  public static final class AutoConstants {}

  public static final class LimelightConstants {
    public static final boolean kUseMegaTag2 = true;
    public static final String kLimelightName = "limelight";
    public static final Vector<N3> kMegaTag2VisionMeasurementStdDevs = VecBuilder.fill(.7, .7, 9999999);
  }

  public static final class Units {
    public static final TimeUnit kTimeUnit = Seconds;
    public static final VoltageUnit kVoltageUnit = Volts;
    public static final DistanceUnit kDistanceUnit = Inches;
    public static final AngleUnit kAngleUnit = Degrees;
    public static final LinearVelocityUnit kLinearVelocityUnit = kDistanceUnit.per(kTimeUnit);
    public static final LinearAccelerationUnit kLinearAccelerationUnit = kLinearVelocityUnit.per(kTimeUnit);
    public static final AngularVelocityUnit kAngularVelocityUnit = kAngleUnit.per(kTimeUnit);
    public static final AngularAccelerationUnit kAngularAccelerationUnit = kAngularVelocityUnit.per(kTimeUnit);
  }
}
