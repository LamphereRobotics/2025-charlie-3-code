// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeStick;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public final class AlgaeStickConstants {
    public static final class Motor {
        public static final int kCanId = 13;
        public static final boolean kInverted = false;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final MotorType kMotorType = MotorType.kBrushless;
    }

    public static final class Encoder {
        public static final Angle kPositionConversion = Degrees.of(9.8);
        public static final AngularVelocity kVelocityConversion = kPositionConversion.per(Minute);
    }

    public static final class PID {
        public static final double kP = 0.044;
        public static final double kI = 0.1;
        public static final double kD = 0;

        public static final Angle kPositionTolerance = Degrees.of(5);
        public static final AngularVelocity kVelocityTolerance = DegreesPerSecond.of(1);
        public static final Angle kIZone = kPositionTolerance;
        public static final Voltage kIntegratorRange = Volts.of(0.3);
    }

    public static final class Positions {
        public static final Angle kHigh = Degrees.of(90);
        public static final Angle kLow = Degrees.of(0);

        public static final Angle kMinPosition = Degrees.of(-5);
        public static final Angle kMaxPosition = Degrees.of(95);
        public static final Angle kStartPosition = Degrees.of(223);

        public static final Angle kToleranceClose = Degrees.of(2);
        public static final Angle kToleranceFar = Degrees.of(15);
        public static final boolean kForwardSoftLimitEnabled = true;
        public static final boolean kReverseSoftLimitEnabled = true;
    }

    public static final class Outputs {
        public static final double kRampRate = 0.25;

        public static final Voltage kUp = Volts.of(4);
        public static final Voltage kDown = Volts.of(-2);
        public static final Voltage kMoveFar = Volts.of(4);
        public static final Voltage kMoveClose = Volts.of(0.5);
    }
}
