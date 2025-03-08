// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.UpperAlgaeArm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;

public class UpperAlgaeArmConstants {
    public static final class ArmMotor {
        public static final int kCanId = 14;
        public static final boolean kInverted = false;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final MotorType kMotorType = MotorType.kBrushless;
    }

    public static final class Encoder {
        public static final Angle kPositionConversion = Degrees.of(17.340);
        public static final AngularVelocity kVelocityConversion = DegreesPerSecond.of(0.0289);
    }

    public static final class Feedforward {
        public static final Voltage kS = Volts.of(1.1);
        public static final Voltage kG = Volts.of(0.0);
        public static final double kV = 0.00816;
    }

    public static final class PID {
        // Coefficients
        public static final Per<VoltageUnit, AngleUnit> kP = Volts.per(Degrees).ofNative(0.05);
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
        public static final AngularVelocity kVelocity = DegreesPerSecond.of(232);
        public static final AngularAcceleration kAcceleration = DegreesPerSecond.per(Second).of(928);
    }

    public static final class Positions {
        public static final Angle kPickup = Degrees.of(40);
        public static final Angle kHold = Degrees.of(90);
        public static final Angle kScore = Degrees.of(90);

        public static final Angle kMinPosition = Degrees.of(35);
        public static final Angle kMaxPosition = Degrees.of(95);
        public static final Angle kStartPosition = Degrees.of(122);

        public static final boolean kForwardSoftLimitEnabled = true;
        public static final boolean kReverseSoftLimitEnabled = true;
    }

    public static final class Outputs {
        public static final double kRampRate = 0.25;

        public static final Voltage kArmUp = Volts.of(2);
        public static final Voltage kArmDown = Volts.of(-2);
    }
}
