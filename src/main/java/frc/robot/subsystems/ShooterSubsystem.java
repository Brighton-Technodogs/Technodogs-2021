/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  private TalonFX bottomShooter = new TalonFX(Constants.ShooterSubsystem.bottomShooterFalconCan);
  private TalonFX rightShooter = new TalonFX(Constants.ShooterSubsystem.rightShooterFalconCan);
  private TalonFX leftShooter = new TalonFX(Constants.ShooterSubsystem.leftShooterFalconCan);
  
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {

  }

  public void shoot (double bottomSpeed, double rightSpeed, double leftSpeed)
  {
    bottomShooter.set(ControlMode.PercentOutput, -bottomSpeed);
    rightShooter.set(ControlMode.PercentOutput, -rightSpeed);
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
      0, //0
      0, //1
      0, //2
      0.5, //3
      0.4, //4
      0.4, //5
      0.4, //6
      0.4, //7
      0.4, //8
      0.4, //9
      0.425, //10
      0.425, //11
      0.435, //12
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
