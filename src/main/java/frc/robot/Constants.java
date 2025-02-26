// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj.TimedRobot;

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
    public static final int kFrontLeftDriveMotorPort = 2;
    public static final int kRearLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 5;

    public static final int kFrontLeftTurningEncoderPorts = 1;
    public static final int kRearLeftTurningEncoderPorts = 4;
    public static final int kFrontRightTurningEncoderPorts = 2;
    public static final int kRearRightTurningEncoderPorts = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final int kGyroPort = 1;

    public static final double kDriveScale = (Units.inchesToMeters(4) * Math.PI) / 6.75;

    // If you call DriveSubsystem.drive() with a different period make sure to
    // update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    public static final double kTrackWidth = 0.57;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.58;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.

    public static final double kMaxSpeedMetersPerSecond = 4.2;
    public static final double kMaxRotationRadiansPerSecond =  Math.PI * 2;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 28.5;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 386.9717;
    public static final double kMaxModuleSpeedMetersPerSecond = 4.2;

    public static final double kPModuleTurningController = 2.0;
    public static final double kIModuleTurningController = 5.0;
    public static final double kDModuleTurningController = 0.0;
    public static final double kSModuleTurningFeedforward = 0.38;
    public static final double kVModuleTurningFeedforward = 0.3961;
    public static final double kPositionToleranceModuleTurningController = 0.1;
    public static final double kVelocityToleranceModuleTurningController = 0.5;
    public static final double kIZoneModuleTurningController = 2.0;
    public static final double kIntegratorMaxModuleTurningController = 2.0;
	
    public static final double kPModuleDriveController = 0.5;
    public static final double kIModuleDriveController = 0.0;
    public static final double kDModuleDriveController = 0.0;
    public static final double kSModuleDriveFeedforward = 0.25156;
    public static final double kVModuleDriveFeedforward = 2.5633;
    public static final double kToleranceModuleDriveController = 0.005;

    public static final int kMaxDriveCurrent = 30;
    public static final int kMaxTurnCurrent = 20;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorStickPort = 1;

    public static final double kDeadband = 0.15;
  }

  public static final class ShooterConstants {
    public static final int kKicky = 9;
    public static final int kKickyLimitSwitch = 6;
    public static final int kShootNSuckUno = 12;
    public static final int kShootNSuckDos = 13;
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

    public static final Distance kMinPosition = Inches.of(25.625);
    public static final Distance kMaxPosition = Inches.of(71);
    public static final Distance kStartPosition = kMinPosition;

    public static final boolean kForwardSoftLimitEnabled = true;
    public static final boolean kReverseSoftLimitEnabled = true;
  }

  public static final class Outputs {
    public static final Voltage kVoltage = Volts.of(2);
  }

  public static final TimeUnit kTimeUnit = Seconds;
  public static final VoltageUnit kVoltageUnit = Volts;
  public static final DistanceUnit kDistanceUnit = Inches;
  public static final LinearVelocityUnit kLinearVelocityUnit = kDistanceUnit.per(kTimeUnit);
  public static final LinearAccelerationUnit kLinearAccelerationUnit = kLinearVelocityUnit.per(kTimeUnit);
}

  public static final class ClimberConstants {
    public static final int kClimbyUno = 0;
    public static final int kClimbyDos = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 14.2985;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 4;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class LimelightConstants {
    public static final boolean kUseMegaTag2 = true;
    public static final String kLimelightName = "limelight";
    public static final Vector<N3> kMegaTag2VisionMeasurementStdDevs = VecBuilder.fill(.7, .7, 9999999);
  }
}
