/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  //create the shooter objects from constants can ID
  private TalonFX bottomShooter = new TalonFX(Constants.ShooterSubsystem.bottomShooterFalconCan);
  private TalonFX rightShooter = new TalonFX(Constants.ShooterSubsystem.rightShooterFalconCan);
  private TalonFX leftShooter = new TalonFX(Constants.ShooterSubsystem.leftShooterFalconCan);
  
  //create objects of sensor information for each shooter
  public final TalonFXSensorCollection bottomShooterSensor;
  public final TalonFXSensorCollection rightShooterSensor;
  public final TalonFXSensorCollection leftShooterSensor;
  //TalonFXConfiguration fxC = new TalonFXConfiguration();

  //hard set PID values
  double pValue = 1;
  double iValue = 0.002;
  double dValue = 50;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() 
  {
    try
    {
      SmartDashboard.getNumber("Array Index", 0);
    }
    catch (Exception e)
    {
      SmartDashboard.putNumber("Array Index", 0);
    }

    // right motor is reversed
    //set right sensor to it's motor and set values
    rightShooterSensor = rightShooter.getSensorCollection();
    rightShooter.configPeakOutputForward(0);
    rightShooter.configPeakOutputReverse(-1);
    rightShooter.config_kP(0, pValue);
    rightShooter.config_kI(0, iValue);
    rightShooter.config_kD(0, dValue);
    rightShooter.config_kF(0, 0.1);

    // bottom motor is reversed
    //set bottom sensor to it's motor and set values
    bottomShooterSensor = bottomShooter.getSensorCollection();
    bottomShooter.configPeakOutputForward(0);
    bottomShooter.configPeakOutputReverse(-1);
    bottomShooter.config_kP(0, pValue);
    bottomShooter.config_kI(0, iValue);
    bottomShooter.config_kD(0, dValue);
    bottomShooter.config_kF(0, 0.1);

    // left motor is not reversed
    //set left sensor to it's motor and set values
    leftShooterSensor = leftShooter.getSensorCollection();
    leftShooter.configPeakOutputForward(1);
    leftShooter.configPeakOutputReverse(0);
    leftShooter.config_kP(0, pValue);
    leftShooter.config_kI(0, iValue);
    leftShooter.config_kD(0, dValue);
    leftShooter.config_kF(0, 0.1);
  }

  //set each motor to desired speed using percent
  public void shoot (double bottomSpeed, double rightSpeed, double leftSpeed)
  {
    bottomShooter.set(ControlMode.PercentOutput, -1*bottomSpeed);
    rightShooter.set(ControlMode.PercentOutput, -1*rightSpeed);
    leftShooter.set(ControlMode.PercentOutput, leftSpeed);
  }

  public void displayEncoders()
  {
    SmartDashboard.putNumber("Bottom Shooter Encoder", bottomShooterSensor.getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Right Shooter Encoder", rightShooterSensor.getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Left Shooter Encoder", leftShooterSensor.getIntegratedSensorVelocity());
  }

  //unused
  int distanceOffset = 5;

  //array for each heights taken everyfoot starting at 5 feet
  int[] heights = 
  {
    46,
    42,
    40,
    36,
    34,
    32,
    30,
    28,
    26,
    25,
    24,
    23,
    22,
    21,
    20,
    19,
    18
  };

  //coordinating speed for each area index
  public final double[] speeds = 
  {
      0.4, //0
      0.4, //1
      0.4, //2
      0.4, //3
      0.3, //4
      0.30, //5
      0.35, //6
      0.275, //7
      0.275, //8
      0.2675, //9
      0.3, //10
      0.3, //11
      0.31, //12
      0.32, //13
      0.32, //14
      0.33, //15
      0.40, //16
      0.42, //17
      0.42, //18
      0.42, //19
      0.42, //20
      0.42, //21
      0.42 //22
  };

  //report index of inputed area
  public double getDistance (double area)
  {
      if (area == 0)
      {
        return 0;
      }
      else if (area < heights[heights.length - 1])
      {
        return heights.length;
      }
      return getDistance(area, 0, heights.length);
  }

  //recursive search for the desired area
  public double getDistance (double area, int start, int end) throws ArrayIndexOutOfBoundsException
  {
    if (Math.abs(start - end) == 1)
    {
        return (end + start) / 2;
    }

    if (heights[(end + start) / 2] == area || (heights[(end + start) / 2] > area && heights[(end + start) / 2 + 1] < area))
    {
        return (end + start) / 2;
    }
    else if (heights[(end + start) / 2] < area)
    {
        return getDistance(area, start, (end + start) / 2);
    }
    else
    {
        return getDistance(area, (end + start) / 2, end);
    }
  }

  //create limelight network table object
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  //get the X Coordinate of target
  public double getXCoord ()
  {
      return limelightTable.getEntry("tx").getDouble(0);
  }
  //get the Y Coordinate of target
  public double getYCoord ()
  {
      return limelightTable.getEntry("ty").getDouble(0);
  }
  //get the Horizontal length of target
  public double getHorizontal ()
  {
      return limelightTable.getEntry("thor").getDouble(0);
  }
  //get the Vertical Height of target
  public double getVertical ()
  {
      return limelightTable.getEntry("tvert").getDouble(0);
  }
  //get the area of the target
  public double getArea ()
  {
      return limelightTable.getEntry("thor").getDouble(0) * limelightTable.getEntry("tvert").getDouble(0);
  }

  //set the shooter motors to desired speed using velocity
  public void SpinToSpeed (double spinSpeed)
  {
    rightShooter.set(ControlMode.Velocity, -1 * spinSpeed); // encoder ticks per 100ms
    bottomShooter.set(ControlMode.Velocity, -1 * spinSpeed);
    leftShooter.set(ControlMode.Velocity, spinSpeed);
    SmartDashboard.putNumber("Current Velocity", rightShooterSensor.getIntegratedSensorVelocity());

    SmartDashboard.putNumber("right Output", rightShooter.getMotorOutputPercent());
    SmartDashboard.putNumber("bottom Output", bottomShooter.getMotorOutputPercent());
    SmartDashboard.putNumber("left Output", leftShooter.getMotorOutputPercent());
  }

  @Override
  public void periodic() 
  {
    
  }
}
