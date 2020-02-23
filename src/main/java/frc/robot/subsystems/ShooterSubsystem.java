/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  private TalonFX bottomShooter = new TalonFX(21);
  private TalonFX rightShooter = new TalonFX(23);
  private TalonFX leftShooter = new TalonFX(22);
  
  public final TalonFXSensorCollection bottomShooterSensor;
  public final TalonFXSensorCollection rightShooterSensor;
  public final TalonFXSensorCollection leftShooterSensor;
  //TalonFXConfiguration fxC = new TalonFXConfiguration();

  double pValue = 1;
  double iValue = 0.002;
  double dValue = 50;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() 
  {
    // right motor is reversed
    rightShooterSensor = rightShooter.getSensorCollection();
    rightShooter.configPeakOutputForward(0);
    rightShooter.configPeakOutputReverse(-1);
    rightShooter.config_kP(0, pValue);
    rightShooter.config_kI(0, iValue);
    rightShooter.config_kD(0, dValue);
    rightShooter.config_kF(0, 0.1);

    // bottom motor is reversed
    bottomShooterSensor = bottomShooter.getSensorCollection();
    bottomShooter.configPeakOutputForward(0);
    bottomShooter.configPeakOutputReverse(-1);
    bottomShooter.config_kP(0, pValue);
    bottomShooter.config_kI(0, iValue);
    bottomShooter.config_kD(0, dValue);
    bottomShooter.config_kF(0, 0.1);

    // left motor is not reversed
    leftShooterSensor = leftShooter.getSensorCollection();
    leftShooter.configPeakOutputForward(1);
    leftShooter.configPeakOutputReverse(0);
    leftShooter.config_kP(0, pValue);
    leftShooter.config_kI(0, iValue);
    leftShooter.config_kD(0, dValue);
    leftShooter.config_kF(0, 0.1);
  }

  public void shoot (double bottomSpeed, double rightSpeed, double leftSpeed)
  {
    bottomShooter.set(ControlMode.PercentOutput, -1*bottomSpeed);
    rightShooter.set(ControlMode.PercentOutput, -1*rightSpeed);
    leftShooter.set(ControlMode.PercentOutput, leftSpeed);
  }

  int distanceOffset = 5;

  int[] areas = 
  {
    4830,
    4141,
    3496,
    2890,
    2508,
    2270,
    1820,
    1708,
    1482,
    1320,
    1248,
    1200,
    1012,
    924,
    840,
    760,
    780,
    760,
    595
  };

  public final double[] speeds = 
  {
      1, //0
      1, //1
      1, //2
      0.5, //3
      0.43, //4
      0.43, //5
      0.43, //6
      0.43, //7
      0.43, //8
      0.43, //9
      0.435, //10
      0.435, //11
      0.44, //12
      0.46, //13
      0.46 //14
  };

  public double getDistance (double area)
  {
      if (area == 0)
      {
        return 0;
      }
      else if (area < areas[areas.length - 1])
      {
        return areas.length;
      }
      return getDistance(area, 0, areas.length);
  }

  public double getDistance (double area, int start, int end) throws ArrayIndexOutOfBoundsException
  {
    if (Math.abs(start - end) == 1)
    {
        return (end + start) / 2;
    }

    if (areas[(end + start) / 2] == area || (areas[(end + start) / 2] > area && areas[(end + start) / 2 + 1] < area))
    {
        return (end + start) / 2;
    }
    else if (areas[(end + start) / 2] < area)
    {
        return getDistance(area, start, (end + start) / 2);
    }
    else
    {
        return getDistance(area, (end + start) / 2, end);
    }
  }

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  public double getXCoord ()
  {
      return limelightTable.getEntry("tx").getDouble(0);
  }
  public double getYCoord ()
  {
      return limelightTable.getEntry("ty").getDouble(0);
  }
  public double getHorizontal ()
  {
      return limelightTable.getEntry("thor").getDouble(0);
  }
  public double getVertical ()
  {
      return limelightTable.getEntry("tvert").getDouble(0);
  }
  public double getArea ()
  {
      return limelightTable.getEntry("thor").getDouble(0) * limelightTable.getEntry("tvert").getDouble(0);
  }

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
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
