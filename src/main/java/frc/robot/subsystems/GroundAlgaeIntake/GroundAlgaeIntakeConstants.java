// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundAlgaeIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Voltage;

public class GroundAlgaeIntakeConstants {
    public static final class Motor {
        public static final int kCanId = 12;
        public static final boolean kInverted = true;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final MotorType kMotorType = MotorType.kBrushless;
    }

    public static final class LimitSwitch {
        public static final int kPort = 9;
    }

    public static final class Outputs {
        public static final Voltage kIn = Volts.of(-12);
        public static final Voltage kOut = Volts.of(12);
        public static final Voltage kHold = Volts.of(-1);
    }
}
