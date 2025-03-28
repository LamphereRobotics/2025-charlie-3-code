// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
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
  public static final class FieldConstants {
    public static final Distance kStartingLine = Inches.of(298.5);
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
    public static final int kZeroGyro = XboxController.Button.kBack.value;
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
    public static final int kEjectCoral = 2;
  }

  public static final class AutoConstants {
  }

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
    public static final PerUnit<AngularVelocityUnit, VoltageUnit> kAngularKVUnit = kAngularVelocityUnit
        .per(kVoltageUnit);
  }
}
