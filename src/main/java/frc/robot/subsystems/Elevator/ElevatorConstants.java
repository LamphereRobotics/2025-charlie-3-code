// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

/** Add your docs here. */
public class ElevatorConstants {
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
		public static final Distance kPositionConversion = Inches.of(0.2154);
		public static final LinearVelocity kVelocityConversion = kPositionConversion.per(Minute);
	}

	public static final class Feedforward {
		public static final Voltage kS = Volts.of(0.024);
		public static final Voltage kG = Volts.of(0.06);
		public static final double kV = 0.585;
	}

	public static final class PID {
		// Coefficients
		// Unit is volts / inch
		public static final Per<VoltageUnit, DistanceUnit> kP = Volts.per(Inch).ofNative(1.5);
		// TODO: Convert to unit type
		// Unit is volts / (inch * second)
		public static final double kI = 0;
		// TODO: Convert to unit type
		// Unit is volts / inch / second
		public static final double kD = 0.15;

		// Extra config
		public static final Distance kPositionTolerance = Inches.of(1);
		public static final LinearVelocity kVelocityTolerance = InchesPerSecond.of(0.25);
		public static final Distance kIZone = Inches.of(0);
		public static final Voltage kIntegratorRange = Volts.of(0);
	}

	public static final class Constraints {
		public static final LinearVelocity kVelocity = InchesPerSecond.of(9.6);
		public static final LinearAcceleration kAcceleration = InchesPerSecond.per(Second).of(24.4);
		public static final double kRampRate = 0.25;
	}

	/**
	 * All positions will be distances from the ground to the [center of the
	 * elevator? end of the outtake?]
	 */
	public static final class Positions {
		public static final Distance kIntake = Inches.of(26.0);
		public static final Distance kL2 = Feet.of(2).plus(Inches.of(7.875));
		public static final Distance kL3 = Feet.of(3).plus(Inches.of(11.625));
		public static final Distance kL4 = Feet.of(6);

		public static final Distance kMinPosition = Inches.of(7.5);
		public static final Distance kMaxPosition = Inches.of(40);
		public static final Distance kStartPosition = kMinPosition;

		public static final boolean kForwardSoftLimitEnabled = true;
		public static final boolean kReverseSoftLimitEnabled = true;
	}

	public static final class Outputs {
		public static final Voltage kUp = Volts.of(12);
		public static final Voltage kDown = Volts.of(-4);
	}

	public static final TimeUnit kTimeUnit = Seconds;
	public static final VoltageUnit kVoltageUnit = Volts;
	public static final DistanceUnit kDistanceUnit = Inches;
	public static final LinearVelocityUnit kLinearVelocityUnit = kDistanceUnit.per(kTimeUnit);
	public static final LinearAccelerationUnit kLinearAccelerationUnit = kLinearVelocityUnit.per(kTimeUnit);
}
