// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPorts,
      DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPorts,
      DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPorts,
      DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPorts,
      DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroPort);

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }
      ,Pose2d.kZero);


  private boolean m_slowMode = false;
  private boolean m_fieldRelative = true;
  private Pose2d m_prevPose = m_poseEstimator.getEstimatedPosition();
  private Translation2d m_velocity = Translation2d.kZero;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    RobotConfig config = new RobotConfig(1, 1, null);
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    useMegaTag2VisionEstimate();
    
    m_velocity = m_poseEstimator.getEstimatedPosition().getTranslation().minus(m_prevPose.getTranslation()).div(0.02);
    m_prevPose = m_poseEstimator.getEstimatedPosition();
  
    m_frontLeft.logStateToDashboard("drive-front-left");
    m_frontRight.logStateToDashboard("drive-front-right");
    m_rearLeft.logStateToDashboard("drive-rear-left");
    m_rearRight.logStateToDashboard("drive-rear-right");
    SmartDashboard.putNumber("drive-gyro-angle", getHeading());
    SmartDashboard.putBoolean("drive-is-slow", m_slowMode);
    SmartDashboard.putBoolean("drive-is-field-relative",
        m_fieldRelative);
	  SmartDashboard.putNumber("drive-pose-x", getPose().getX());
	  SmartDashboard.putNumber("drive-pose-y", getPose().getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command driveTeleop(CommandXboxController driveController) {
    return new RunCommand(() -> {
      double leftY = WithDeadband(OIConstants.kDeadband,
          -driveController.getLeftY());
      double leftX = WithDeadband(OIConstants.kDeadband,
          -driveController.getLeftX());
      double rightX = WithDeadband(OIConstants.kDeadband,
          -driveController.getRightX());
      double maxSpeed = m_slowMode
          ? DriveConstants.kMaxSpeedMetersPerSecond * 0.5
          : DriveConstants.kMaxSpeedMetersPerSecond;
      drive(
          // Multiply by max speed to map the
          // joystick
          // unitless inputs to actual units.
          // This will map the [-1, 1] to [max
          // speed
          // backwards, max speed forwards],
          // converting them to actual units.
          leftY * maxSpeed,
          leftX * maxSpeed,
          rightX * DriveConstants.kMaxRotationRadiansPerSecond,
          m_fieldRelative);
    }, this);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    System.out.println("reset gyro");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public AngularVelocity getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValue().times(DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public Command resetGyro() {
    return new InstantCommand(this::zeroHeading);
  }

  public boolean getSlowMode() {
    return m_slowMode;
  }

  public Command setSlowModeCommand(boolean slowMode) {
    return new InstantCommand(() -> m_slowMode = slowMode);
  }

  public boolean getFieldRelative() {
    return m_slowMode;
  }

  public Command setFieldRelativeCommand(boolean fieldRelative) {
    return new InstantCommand(() -> m_fieldRelative = fieldRelative);
  }
  
  private static double WithDeadband(double deadband, double thumbstick) {
    return Math.abs(thumbstick) < deadband ? 0
        : ((Math.abs(thumbstick) - deadband) / (1 - deadband) * Math.signum(thumbstick));
  }

  private ChassisSpeeds getRobotRelativeSpeeds(){
    var speeds = new ChassisSpeeds(m_velocity.getX(), m_velocity.getY(), getTurnRate().in(RadiansPerSecond));
    return speeds; 
  }
  
  private void driveRobotRelative(ChassisSpeeds speed){

  }

  private void useMegaTag2VisionEstimate() {
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation(LimelightConstants.kLimelightName,
        m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kLimelightName);
    if (Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                          // vision updates
    {
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(LimelightConstants.kMegaTag2VisionMeasurementStdDevs);
      m_poseEstimator.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    }
  }

}