/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
  private final String moduleIdentifier;
  private final TalonFX m_driveMotor;
  private final VictorSPX m_twistMotor;

  // private boolean rotationEnabled = false;

  private final TalonFXSensorCollection m_driveMotorSensors;
  private final AnalogPotentiometer m_twistEncoder;

  private double offset;

  private ShuffleboardTab subsystemShuffleboardTab;

  private NetworkTableEntry sbSwerveModuleAngleCommand;
  private NetworkTableEntry sbSwerveModuleAngleActual;

  private NetworkTableEntry sbSwerveModuleSpeedCommand;
  private NetworkTableEntry sbSwerveModuleSpeedActual;

  private NetworkTableEntry sbSwerveModuleSpeedRawCommand;
  private NetworkTableEntry sbSwerveModuleSpeedRawActual;

  private NetworkTableEntry sbSwerveModuleTurnMotorOutput;



  private final PIDController m_drivePIDController =
      new PIDController(Constants.DriveSubsystem.kSwerveDrivePID_P, 
                        Constants.DriveSubsystem.kSwerveDrivePID_I,
                        Constants.DriveSubsystem.kSwerveDrivePID_D);

  //Using a TrapezoidProfile PIDController to allow for smooth turning
  private final PIDController m_twistPIDController
      = new PIDController(
        Constants.DriveSubsystem.kSwerveTwistPID_P,
        Constants.DriveSubsystem.kSwerveTwistPID_I,
        Constants.DriveSubsystem.kSwerveTwistPID_D);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorCanID   ID for the drive motor.
   * @param twistMotorCanID ID for the turning motor.
   */
  public SwerveModule(int driveMotorCanID,
                      int twistMotorCanID,
                      int twistEncoderPort,
                      double offset,
                      ShuffleboardTab tab,
                      String moduleIdentifier) {

    // TODO: Add offsets

    System.out.println("Initializing Swerve: " + moduleIdentifier + ". Driver motor = " + driveMotorCanID + ". Turning motor = " + twistMotorCanID);


    // use this to put to individual module dashboard.
    subsystemShuffleboardTab = Shuffleboard.getTab(moduleIdentifier);

    // use this to put to main subsystem dashboard.
    // subsystemShuffleboardTab = tab;


    sbSwerveModuleAngleCommand = subsystemShuffleboardTab.add(moduleIdentifier + "_degC", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 360))
    .getEntry();


    sbSwerveModuleAngleActual = subsystemShuffleboardTab.add(moduleIdentifier + "_degA", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 360))
    .getEntry();

    sbSwerveModuleSpeedCommand = subsystemShuffleboardTab.add(moduleIdentifier + "_Vc", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -10, "max", 10))
    .getEntry();

    sbSwerveModuleSpeedActual = subsystemShuffleboardTab.add(moduleIdentifier + "_Va", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -10, "max", 10))
    .getEntry();

    sbSwerveModuleSpeedRawCommand = subsystemShuffleboardTab.add(moduleIdentifier + "_R_Vc", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -22000, "max", 22000))
    .getEntry();

    sbSwerveModuleSpeedRawActual = subsystemShuffleboardTab.add(moduleIdentifier + "_R_Va", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -22000, "max", 22000))
    .getEntry();

    sbSwerveModuleTurnMotorOutput = subsystemShuffleboardTab.add(moduleIdentifier + "twistOut", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    m_driveMotor = new TalonFX(driveMotorCanID);
    m_twistMotor = new VictorSPX(twistMotorCanID);
    this.offset = offset;
    this.moduleIdentifier = moduleIdentifier;

    this.m_driveMotorSensors = m_driveMotor.getSensorCollection(); //new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);

    m_driveMotor.configPeakOutputForward(1);
    m_driveMotor.configPeakOutputReverse(-1);
    m_driveMotor.config_kP(0, Constants.DriveSubsystem.kSwerveDrivePID_P);
    m_driveMotor.config_kI(0, Constants.DriveSubsystem.kSwerveDrivePID_I);
    m_driveMotor.config_kD(0, Constants.DriveSubsystem.kSwerveDrivePID_D);
    m_driveMotor.config_kF(0, Constants.DriveSubsystem.kSwerveDrivePID_F);
    // m_driveMotor.configAllowableClosedloopError(0, 0);
    // m_driveMotor.configMaxIntegralAccumulator(0, maxIntegralAccumulator);
    // m_driveMotor.configClosedLoopPeriod(0, PIDLoopRate);

    this.m_twistEncoder = new AnalogPotentiometer(twistEncoderPort, 360.0, 0.0);

    // Limit the PID Controller's input range between 0 and 360 and set the input to be continuous.
    m_twistPIDController.enableContinuousInput(0, 360);
    m_twistPIDController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      convertTicksPerTimeUnitToMetersPerSecond(m_driveMotorSensors.getIntegratedSensorVelocity()), 
      Rotation2d.fromDegrees(m_twistEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.

    double setpoint, setpoint_scaled;

    // Our encoders are not aligned such that 0 means "the front of the robot".
    // Because of this, we need to add the offset here
    setpoint = state.angle.getDegrees() + this.offset;

    // Because we added an offset, we now have to normalize the angle to 0-360
    if (setpoint < 0) {
      setpoint_scaled =  360 - (setpoint * -1);
    } else if (setpoint > 360) {
      setpoint_scaled = setpoint - 360;
    } else {
      setpoint_scaled = setpoint;
    }

    sbSwerveModuleAngleCommand.setDouble(setpoint_scaled);

    double currentAngle = m_twistEncoder.get();
    double currentAngle_scaled;

    // display the actual angle of the wheel on shuffleboard.
    currentAngle -= this.offset;

    if (currentAngle < 0) {
      currentAngle_scaled =  360 - (currentAngle * -1);
    } else if (currentAngle > 360) {
      currentAngle_scaled = currentAngle - 360;
    } else {
      currentAngle_scaled = currentAngle;
    }

    sbSwerveModuleAngleActual.setDouble(currentAngle_scaled);

    sbSwerveModuleSpeedCommand.setDouble(state.speedMetersPerSecond);
    sbSwerveModuleSpeedActual.setDouble(convertTicksPerTimeUnitToMetersPerSecond(m_driveMotorSensors.getIntegratedSensorVelocity()));

        // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_twistPIDController.calculate(
          m_twistEncoder.get(), setpoint_scaled
      );

      sbSwerveModuleTurnMotorOutput.setDouble(turnOutput);

    // System.out.println("Setting State. Drive Motor Output  = " + driveOutput + ". Turning motor output = " + turnOutput);

    // Convert the motor output command to ticks/100ms
    // TODO: Check that this works.
    double driveOutput = convertMetersPerSecondToTicksPerTimeUnit(state.speedMetersPerSecond);

    sbSwerveModuleSpeedRawCommand.setDouble(driveOutput);
    sbSwerveModuleSpeedRawActual.setDouble(m_driveMotorSensors.getIntegratedSensorVelocity());
    m_driveMotor.set(ControlMode.Velocity, driveOutput);
    m_twistMotor.set(ControlMode.PercentOutput, turnOutput);
  }

  private double convertTicksToMeters(double ticks){

    double wheelCircumference = Constants.DriveSubsystem.kWheelDiameter * Math.PI;
    double distancePerMotorRevolution = wheelCircumference / Constants.DriveSubsystem.kGearRatioMotorToWheel;
    double finalRatio = distancePerMotorRevolution / Constants.DriveSubsystem.kMotorEncoderTicksPerRev;

    return ticks*finalRatio;
  }

  private double convertMetersToTicks(double meters){

    double wheelCircumference = Constants.DriveSubsystem.kWheelDiameter * Math.PI;
    double motorRevolutionsPerDistance = Constants.DriveSubsystem.kGearRatioMotorToWheel/wheelCircumference;
    double finalRatio = Constants.DriveSubsystem.kMotorEncoderTicksPerRev * motorRevolutionsPerDistance;

    return meters * finalRatio;
  
  }

  private double convertTicksPerTimeUnitToMetersPerSecond(double ticksPer100ms){
    return convertTicksToMeters(ticksPer100ms) / Constants.DriveSubsystem.kMotorEncoderTimeUnit;
  }

  private double convertMetersPerSecondToTicksPerTimeUnit(double metersPerSec){
    return convertMetersToTicks(metersPerSec) * Constants.DriveSubsystem.kMotorEncoderTimeUnit;
  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
  }

  public double getRawAngle() {
    return this.m_twistEncoder.get();
  }

  // public void enableRotation(){
  //   rotationEnabled = true;

  // }

  // public void disableRotation(){
  //   rotationEnabled = false;
  // }
}