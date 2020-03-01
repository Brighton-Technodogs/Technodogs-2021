/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearRight;

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  SwerveDriveKinematics mDriveKinematics =
  new SwerveDriveKinematics(
    new Translation2d(Constants.DriveSubsystem.kWheelBase / 2, Constants.DriveSubsystem.kTrackWidth / 2),
    new Translation2d(Constants.DriveSubsystem.kWheelBase / 2, -Constants.DriveSubsystem.kTrackWidth / 2),
    new Translation2d(-Constants.DriveSubsystem.kWheelBase / 2, Constants.DriveSubsystem.kTrackWidth / 2),
    new Translation2d(-Constants.DriveSubsystem.kWheelBase / 2, -Constants.DriveSubsystem.kTrackWidth / 2));

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(mDriveKinematics, getAngle());

  private ShuffleboardTab subsystemShuffleboardTab = Shuffleboard.getTab("Drive Subsystem");

    // Shuffleboard 
    private NetworkTableEntry sbdirectionX = subsystemShuffleboardTab.add("direction X", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbdirectionY = subsystemShuffleboardTab.add("direction Y", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbrotation = subsystemShuffleboardTab.add("rotation", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    // private NetworkTableEntry sbDriveGyro = subsystemShuffleboardTab.add((Sendable) m_gyro)
    // .withWidget(BuiltInWidgets.kGyro)
    // .getEntry();

    private NetworkTableEntry sFieldRelative = subsystemShuffleboardTab.add("fieldOriented", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    subsystemShuffleboardTab.add((Sendable) m_gyro);

    m_frontLeft = new SwerveModule(Constants.DriveSubsystem.kFrontLeftDriveMotorCanID,
                                    Constants.DriveSubsystem.kFrontLeftTwistMotorCanID,
                                    Constants.DriveSubsystem.kFrontLeftEncoderPort,
                                    subsystemShuffleboardTab,
                                    "FL");

    m_rearLeft = new SwerveModule(Constants.DriveSubsystem.kRearLeftDriveMotorCanID,
                                    Constants.DriveSubsystem.kRearLeftTwistMotorCanID,
                                    Constants.DriveSubsystem.kRearLeftEncoderPort,
                                    subsystemShuffleboardTab,
                                    "RL");
    
    m_frontRight = new SwerveModule(Constants.DriveSubsystem.kFrontRightDriveMotorCanID,
                                    Constants.DriveSubsystem.kFrontRightTwistMotorCanID,
                                    Constants.DriveSubsystem.kFrontRightEncoderPort,
                                    subsystemShuffleboardTab,
                                    "FR");

    m_rearRight = new SwerveModule(Constants.DriveSubsystem.kRearRightDriveMotorCanID,
                                    Constants.DriveSubsystem.kRearRightTwistMotorCanID,
                                    Constants.DriveSubsystem.kRearRightEncoderPort,
                                    subsystemShuffleboardTab,
                                    "RR");
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot from the gyro.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(m_gyro.getAngle() * (Constants.DriveSubsystem.kGyroReversed ? 1.0 : -1.0));
  }

  @Override
  public void periodic() {
    // sbDriveGyro.setValue(m_gyro.getAngle());
    // Update the odometry in the periodic block
    m_odometry.update(new Rotation2d(getHeading()), m_frontLeft.getState(), m_rearLeft.getState(),
        m_frontRight.getState(), m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getAngle());
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
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    sbdirectionX.setDouble(xSpeed);
    sbdirectionY.setDouble(ySpeed);
    sbrotation.setDouble(rot);
    sFieldRelative.setBoolean(fieldRelative);

    var swerveModuleStates = mDriveKinematics
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.DriveSubsystem.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveSubsystem.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.DriveSubsystem.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.DriveSubsystem.kGyroReversed ? -1.0 : 1.0);
  }


//   public void disableFrontRightWheelRotation(){
//     m_frontRight.disableRotation();
// }
// public void disableFrontLeftWheelRotation(){
//   m_frontLeft.disableRotation();
// }
// public void disableRearRightWheelRotation(){
//   m_rearRight.disableRotation();
// }
// public void disableRearLeftWheelRotation(){
//   m_rearLeft.disableRotation();
// }
// public void enableFrontRightWheelRotation(){
//   m_frontRight.enableRotation();
// }
// public void enableFrontLeftWheelRotation(){
//   m_frontLeft.enableRotation();
// }
// public void enableRearRightWheelRotation(){
//   m_rearRight.enableRotation();
// }
// public void enableRearLeftWheelRotation(){
//   m_rearLeft.enableRotation();
// }

}
