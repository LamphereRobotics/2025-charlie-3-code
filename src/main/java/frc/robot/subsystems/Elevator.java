// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class ElevatorConfig {
  // Feedforward
  public static final double kSVolts = 0.16;
  public static final double kGVolts = 0.65;
  public static final double kVVoltSecondPerRad = 0.35;
  public static final double kAVoltSecondSquaredPerRad = 0.0;
  public static final double kI = 0;
  public static final double kP = 0;
  public static final double kD = 0;
  public static final double kMaxDegreesPerSecond = 0;
  public static final double kMaxDegreesPerSecondSquared = 0;
  public static final double kStoreArmDegrees = 0;
  public static final double kPositionToleranceDegrees = 0;
  public static final double kVelocityToleranceDegreesPerSecond = 0;
  public static final double kIZone = 0;
  public static final int kIntegratorRange = 0;
  public static final boolean kMotorZweiInverted = false;
  public static final boolean kMotorEinsInverted = false;
  public static final boolean kEncoderEinsInverted = false;
}

public class Elevator extends SubsystemBase {

  private final SparkMax m_motorEins = new SparkMax(ElevatorConstants.kElevatorEins,
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final SparkMax m_motorZwei = new SparkMax(ElevatorConstants.kElevatorZwei,
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motorEins.getEncoder();
  private final ElevatorFeedforward m_feedForward = new ElevatorFeedforward(ElevatorConfig.kSVolts,
      ElevatorConfig.kGVolts,
      ElevatorConfig.kVVoltSecondPerRad,
      ElevatorConfig.kAVoltSecondSquaredPerRad);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      ElevatorConfig.kP,
      ElevatorConfig.kI,
      ElevatorConfig.kD,
      // The motion profile constraints
      new TrapezoidProfile.Constraints(ElevatorConfig.kMaxDegreesPerSecond,
          ElevatorConfig.kMaxDegreesPerSecondSquared));

  /** Creates a new Elevator. */
  public Elevator() {
    SparkMaxConfig MotorEinsConfig = new SparkMaxConfig();
    SparkMaxConfig MotorZweiConfig = new SparkMaxConfig();

    m_controller.setTolerance(ElevatorConfig.kPositionToleranceDegrees,
        ElevatorConfig.kVelocityToleranceDegreesPerSecond);
    m_controller.setIZone(ElevatorConfig.kIZone);
    m_controller.setIntegratorRange(-ElevatorConfig.kIntegratorRange, ElevatorConfig.kIntegratorRange);

    MotorEinsConfig.inverted(ElevatorConfig.kMotorEinsInverted);
    MotorZweiConfig.inverted(ElevatorConfig.kMotorZweiInverted);

    MotorEinsConfig.encoder.inverted(ElevatorConfig.kEncoderEinsInverted);
    m_encoder.setPosition(0);

    m_motorEins.configure(MotorEinsConfig, null, null);
    m_motorZwei.configure(MotorZweiConfig, null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed) {
    // move the motors the right direction with + being up and - being down
    m_motorEins.set(speed);
    m_motorZwei.set(speed);
  }

  public void moveUp() {
    move(0.05);
  }

  public void moveDown() {
    // call move with a - value
    move(-0.05);
  }
}
