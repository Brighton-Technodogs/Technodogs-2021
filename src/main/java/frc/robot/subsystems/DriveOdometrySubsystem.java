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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveOdometrySubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearRight;

  private double directionStickDeadZone = 0.08;
  private double rotateStickDeadZone = 0.08;

  private double foZeroOffset = 0;
  private Boolean fieldOrientedOn = false;

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  private Field2d v_Field2d = new Field2d();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.DriveSubsystem.kDriveKinematics, getAngle());

  private ShuffleboardTab subsystemShuffleboardTab = Shuffleboard.getTab("Drive Odometry Subsystem");

    // Shuffleboard 
    private NetworkTableEntry sbFowardInput = subsystemShuffleboardTab.add("Forward Input", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbSidewaysInput = subsystemShuffleboardTab.add("Sideways Input", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbrotation = subsystemShuffleboardTab.add("rotation", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbPoseX = subsystemShuffleboardTab.add("pose x", 0).getEntry();

    private NetworkTableEntry sbPoseY = subsystemShuffleboardTab.add("pose y", 0).getEntry();

    private NetworkTableEntry sbPoseRotation = subsystemShuffleboardTab.add("pose rotation", 0).getEntry();

    // private NetworkTableEntry sbDriveGyro = subsystemShuffleboardTab.add((Sendable) m_gyro)
    // .withWidget(BuiltInWidgets.kGyro)
    // .getEntry();

    private NetworkTableEntry sFieldRelative = subsystemShuffleboardTab.add("fieldOriented", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

    private NetworkTableEntry sbFLEncoderRaw = subsystemShuffleboardTab.add("FLEncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbFREncoderRaw = subsystemShuffleboardTab.add("FREncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbRLEncoderRaw = subsystemShuffleboardTab.add("RLEncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbRREncoderRaw = subsystemShuffleboardTab.add("RREncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbFLDriveTemp = subsystemShuffleboardTab.add("FL Drive Temp", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 120))
    .getEntry();

    private NetworkTableEntry sbRLDriveTemp = subsystemShuffleboardTab.add("RL Drive Temp", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 120))
    .getEntry();

    private NetworkTableEntry sbFRDriveTemp = subsystemShuffleboardTab.add("FR Drive Temp", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 120))
    .getEntry();

    private NetworkTableEntry sbRRDriveTemp = subsystemShuffleboardTab.add("RR Drive Temp", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 120))
    .getEntry();

    private NetworkTableEntry sbDriveAligned = subsystemShuffleboardTab.add("Drive Aligned", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

  /**
   * Creates a new DriveOdometrySubsystem.
   */
  public DriveOdometrySubsystem() {

    subsystemShuffleboardTab.add((Sendable) m_gyro);
    resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

    m_frontLeft = new SwerveModule(Constants.DriveSubsystem.kFrontLeftDriveMotorCanID,
                                    Constants.DriveSubsystem.kFrontLeftTwistMotorCanID,
                                    Constants.DriveSubsystem.kFrontLeftEncoderPort,
                                    Constants.DriveSubsystem.kFrontLeftEncoderOffset,
                                    subsystemShuffleboardTab,
                                    "FL");

    m_rearLeft = new SwerveModule(Constants.DriveSubsystem.kRearLeftDriveMotorCanID,
                                    Constants.DriveSubsystem.kRearLeftTwistMotorCanID,
                                    Constants.DriveSubsystem.kRearLeftEncoderPort,
                                    Constants.DriveSubsystem.kRearLeftEncoderOffset,
                                    subsystemShuffleboardTab,
                                    "RL");
    
    m_frontRight = new SwerveModule(Constants.DriveSubsystem.kFrontRightDriveMotorCanID,
                                    Constants.DriveSubsystem.kFrontRightTwistMotorCanID,
                                    Constants.DriveSubsystem.kFrontRightEncoderPort,
                                    Constants.DriveSubsystem.kFrontRightEncoderOffset,
                                    subsystemShuffleboardTab,
                                    "FR");

    m_rearRight = new SwerveModule(Constants.DriveSubsystem.kRearRightDriveMotorCanID,
                                    Constants.DriveSubsystem.kRearRightTwistMotorCanID,
                                    Constants.DriveSubsystem.kRearRightEncoderPort,
                                    Constants.DriveSubsystem.kRearRightEncoderOffset,
                                    subsystemShuffleboardTab,
                                    "RR");

    subsystemShuffleboardTab.add(v_Field2d);
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot from the gyro.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(m_gyro.getAngle() * (Constants.DriveSubsystem.kGyroReversed ? 1.0 : -1.0) + (this.fieldOrientedOn ? foZeroOffset : 0));
  }

  public Rotation2d getNZAngle() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() * (Constants.DriveSubsystem.kGyroReversed ? 1.0 : -1.0));
  }

  public Rotation2d getFOAngle() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() * (Constants.DriveSubsystem.kGyroReversed ? 1.0 : -1.0) + 270);
  }

  @Override
  public void periodic() {

    sbFLEncoderRaw.setDouble(m_frontLeft.getRawAngle());
    sbFREncoderRaw.setDouble(m_frontRight.getRawAngle());
    sbRLEncoderRaw.setDouble(m_rearLeft.getRawAngle());
    sbRREncoderRaw.setDouble(m_rearRight.getRawAngle());

    sendSwerveModuleTempsToShuffleboard();

    sbPoseX.setDouble(m_odometry.getPoseMeters().getTranslation().getX());
    sbPoseY.setDouble(m_odometry.getPoseMeters().getTranslation().getY());
    sbPoseRotation.setDouble(m_odometry.getPoseMeters().getRotation().getDegrees());
    v_Field2d.setRobotPose(m_odometry.getPoseMeters().getTranslation().getX(), m_odometry.getPoseMeters().getTranslation().getY(), m_odometry.getPoseMeters().getRotation());
    // sbDriveGyro.setValue(m_gyro.getAngle());
    // Update the odometry in the periodic block

    // Are the motors in the right order? They were not, but they are now
    m_odometry.update(new Rotation2d(java.lang.Math.toRadians(getHeading())), m_frontLeft.getState(), m_frontRight.getState(),
        m_rearLeft.getState(), m_rearRight.getState());

        super.periodic();
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
  
  public void drive(double forwardInput, double sidewaysInput, double rot, boolean fieldRelative) {

    sbFowardInput.setDouble(forwardInput);
    sbSidewaysInput.setDouble(sidewaysInput);
    sbrotation.setDouble(rot);
    sFieldRelative.setBoolean(fieldRelative);

    forwardInput *= Constants.DriveSubsystem.kMaxSpeedMetersPerSecond;
    sidewaysInput *= Constants.DriveSubsystem.kMaxSpeedMetersPerSecond;

    if (forwardInput < directionStickDeadZone 
        && forwardInput > directionStickDeadZone*-1)
    {
      forwardInput = 0;
    }

    if (sidewaysInput < directionStickDeadZone 
        && sidewaysInput > directionStickDeadZone*-1)
    {
      sidewaysInput = 0;
    }

    if (rot < rotateStickDeadZone 
        && rot > rotateStickDeadZone*-1) 
    {
      rot = 0;
    }
    rot *= Constants.DriveSubsystem.kMaxTwistAngularVelocity;
    // Rotation speeds are in radians per second for some reason...
    var swerveModuleStates = Constants.DriveSubsystem.kDriveKinematics
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardInput, sidewaysInput, rot, getFOAngle())
            : new ChassisSpeeds(forwardInput, sidewaysInput, rot));

    // System.out.println("Raw " + swerveModuleStates[0].speedMetersPerSecond);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveSubsystem.kMaxSpeedMetersPerSecond);
    // System.out.println("Normalized " + swerveModuleStates[0].speedMetersPerSecond);


    // m_frontLeft.setDesiredState(
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0))
    // );
    // m_frontRight.setDesiredState(
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0))
    // );
    // m_rearLeft.setDesiredState(
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0))
    // );
    // m_rearRight.setDesiredState(
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0))
    // );

    if ((forwardInput < directionStickDeadZone 
        && forwardInput > directionStickDeadZone*-1) 
        && (sidewaysInput < directionStickDeadZone 
        && sidewaysInput > directionStickDeadZone*-1)
        && (rot < rotateStickDeadZone 
        && rot > rotateStickDeadZone*-1)) 
    {
      m_frontLeft.setDesiredState(swerveModuleStates[2], true); //2
      m_frontRight.setDesiredState(swerveModuleStates[0], true); //0
      m_rearLeft.setDesiredState(swerveModuleStates[3], true); //3
      m_rearRight.setDesiredState(swerveModuleStates[1], true); //1
    }

    else{
      m_frontLeft.setDesiredState(swerveModuleStates[2], false); //2
      m_frontRight.setDesiredState(swerveModuleStates[0], false); //0
      m_rearLeft.setDesiredState(swerveModuleStates[3], false); //3
      m_rearRight.setDesiredState(swerveModuleStates[1], false); //1

    }
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveSubsystem.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0], false);
    m_frontRight.setDesiredState(desiredStates[1], false);
    m_rearLeft.setDesiredState(desiredStates[2], false);
    m_rearRight.setDesiredState(desiredStates[3], false);
  }

  public void sendSwerveModuleTempsToShuffleboard() {
    sbFLDriveTemp.setDouble(m_frontLeft.getDriveTemperature());
    sbRLDriveTemp.setDouble(m_rearLeft.getDriveTemperature());
    sbFRDriveTemp.setDouble(m_frontRight.getDriveTemperature());
    sbRRDriveTemp.setDouble(m_rearRight.getDriveTemperature());
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
   * Zeroes the heading of the robot. Please note that this will break odometry, so it is not inteded to be used during competition
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void zeroFO () {
    // TODO: check that this doesn't break field oriented mode
    this.foZeroOffset = (m_gyro.getAngle() % 360); // normalize gyro angle to 0-360 as to not cause any problems
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in Radians, with 0 being the last reset point
   */
  public double getHeading() {
    // return 0;
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.DriveSubsystem.kGyroReversed ? -1.0 : 1.0) * (Math.PI / 180) ;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.DriveSubsystem.kGyroReversed ? -1.0 : 1.0);
  }

  public void setAligned() {
    sbDriveAligned.setBoolean(true);
  }

  public void unsetAligned() {
    sbDriveAligned.setBoolean(false);
  }

}