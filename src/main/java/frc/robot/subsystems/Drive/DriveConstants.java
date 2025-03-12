// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import frc.robot.Constants.FieldConstants;

public final class DriveConstants {
	public static final double kMaxSpeedMetersPerSecond = 4.2;

	public static final class Positions {
		public static final Pose2d kStartingPose = new Pose2d(FieldConstants.kStartingLine, Inches.of(158.5),
				Rotation2d.kZero);

		public static final Angle kProcessorHeading = Degrees.of(90);
	}
}
