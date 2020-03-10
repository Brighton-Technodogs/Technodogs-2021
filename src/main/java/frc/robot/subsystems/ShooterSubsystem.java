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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  double pValue = 0.06;
  double iValue = 0.02;
  double dValue = 0.8;
  double fValue = 0.05;
  int allowableError = 150;
  int PIDLoopRate = 10; //In ms
  int maxIntegralAccumulator = 1000;

  double bottomSpin = 4; //1,25


  private ShuffleboardTab subsystemShuffleboardTab = Shuffleboard.getTab("Shooter Subsystem");
  private NetworkTableEntry sbpValue = subsystemShuffleboardTab.add("PID pValue", pValue)
  .withWidget(BuiltInWidgets.kDial)
  .withProperties(Map.of("min", 0, "max", 1)) 
  .getEntry();

  private NetworkTableEntry sbiValue = subsystemShuffleboardTab.add("PID iValue", iValue)
  .withWidget(BuiltInWidgets.kDial)
  .withProperties(Map.of("min", 0, "max", 1)) 
  .getEntry();

  private NetworkTableEntry sbdValue = subsystemShuffleboardTab.add("PID dValue", dValue)
  .withWidget(BuiltInWidgets.kDial)
  .withProperties(Map.of("min", 0, "max", 1)) 
  .getEntry();

  private NetworkTableEntry sbfValue = subsystemShuffleboardTab.add("PID fValue", fValue)
  .withWidget(BuiltInWidgets.kDial)
  .withProperties(Map.of("min", 0, "max", 1)) 
  .getEntry();
  
  private NetworkTableEntry sbPIDLoopRate = subsystemShuffleboardTab.add("PID PIDLoopRate ms", PIDLoopRate)
  .withWidget(BuiltInWidgets.kDial)
  .withProperties(Map.of("min", 0, "max",30)) 
  .getEntry();

  private NetworkTableEntry sbmaxIntegralAccumulator = subsystemShuffleboardTab.add("maxIntegralAccumulator", 1000)
  .withWidget(BuiltInWidgets.kDial)
  .withProperties(Map.of("min", 500, "max",2000)) 
  .getEntry();
  
  private NetworkTableEntry sbbottomSpin = subsystemShuffleboardTab.add("bottomSpin", bottomSpin)
  .withWidget(BuiltInWidgets.kDial)
  .withProperties(Map.of("min",0, "max",3)) 
  .getEntry();
  
  
  private NetworkTableEntry llXCoord = subsystemShuffleboardTab.add("LL X Coord.", 0).getEntry();
  private NetworkTableEntry llYCoord = subsystemShuffleboardTab.add("LL Y Coord.", 0).getEntry();
  private NetworkTableEntry llHoriz = subsystemShuffleboardTab.add("LL Horiz.", 0).getEntry();
  private NetworkTableEntry llVert = subsystemShuffleboardTab.add("LL Vert.", 0).getEntry();
  private NetworkTableEntry llArea = subsystemShuffleboardTab.add("LL Area", 0).getEntry();
  private NetworkTableEntry shooterRV = subsystemShuffleboardTab.add("Right Shooter Set Speed", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
  private NetworkTableEntry shooterBV = subsystemShuffleboardTab.add("Bottom Shooter Set Speed", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
  private NetworkTableEntry shooterLV = subsystemShuffleboardTab.add("Left Shooter Set Speed", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();

  private NetworkTableEntry shooterRS = subsystemShuffleboardTab.add("Right Shooter Encoder", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
  private NetworkTableEntry shooterBS = subsystemShuffleboardTab.add("Bottom Shooter Encoder", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
private NetworkTableEntry shooterLS = subsystemShuffleboardTab.add("Left Shooter Encoder", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();

private NetworkTableEntry gshooterRV = subsystemShuffleboardTab.add("Right Shooter Set Speed Graph", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
private NetworkTableEntry gshooterBV = subsystemShuffleboardTab.add("Bottom Shooter Set Speed Graph", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
private NetworkTableEntry gshooterLV = subsystemShuffleboardTab.add("Left Shooter Set Speed Graph", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
private NetworkTableEntry gshooterRS = subsystemShuffleboardTab.add("Graph Right Shooter Encoder", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
private NetworkTableEntry gshooterBS = subsystemShuffleboardTab.add("Graph Bottom Shooter Encoder", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();
private NetworkTableEntry gshooterLS = subsystemShuffleboardTab.add("Graph Left Shooter Encoder", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() 
  {

    sbpValue.setDouble(pValue);
    sbiValue.setDouble(iValue);
    sbdValue.setDouble(dValue);
    sbfValue.setDouble(fValue);
    sbmaxIntegralAccumulator.setNumber(maxIntegralAccumulator);
    sbPIDLoopRate.setNumber(PIDLoopRate);
    sbbottomSpin.setDouble(bottomSpin);
    try
    {
      SmartDashboard.getNumber("Array Index", 0);
    }
    catch (Exception e)
    {
      SmartDashboard.putNumber("Array Index", 0);
    }
    
    SmartDashboard.putNumber("P Value Right", 0);
    SmartDashboard.putNumber("I Value Right", 0.02);
    SmartDashboard.putNumber("D Value Right", 0);
    SmartDashboard.putNumber("F Value Right", 0.05);

    // right motor is reversed
    //set right sensor to it's motor and set values
    rightShooterSensor = rightShooter.getSensorCollection();
    rightShooter.configPeakOutputForward(1);
    rightShooter.configPeakOutputReverse(0);
    rightShooter.config_kP(0, pValue);
    rightShooter.config_kI(0, iValue);
    rightShooter.config_kD(0, dValue);
    rightShooter.config_kF(0, fValue);
    rightShooter.configAllowableClosedloopError(0, 0);
    rightShooter.configMaxIntegralAccumulator(0, maxIntegralAccumulator);
    rightShooter.configClosedLoopPeriod(0, PIDLoopRate);

    // bottom motor is reversed
    //set bottom sensor to it's motor and set values
    bottomShooterSensor = bottomShooter.getSensorCollection();
    bottomShooter.configPeakOutputForward(0);
    bottomShooter.configPeakOutputReverse(-1);
    bottomShooter.config_kP(0, pValue);
    bottomShooter.config_kI(0, iValue);
    bottomShooter.config_kD(0, dValue);
    bottomShooter.config_kF(0, fValue);
    bottomShooter.configAllowableClosedloopError(0, 0);
    bottomShooter.configMaxIntegralAccumulator(0, maxIntegralAccumulator);
    bottomShooter.configClosedLoopPeriod(0, PIDLoopRate);

    // left motor is not reversed
    //set left sensor to it's motor and set values
    leftShooterSensor = leftShooter.getSensorCollection();
    leftShooter.configPeakOutputForward(0);
    leftShooter.configPeakOutputReverse(-1);
    leftShooter.config_kP(0, pValue);
    leftShooter.config_kI(0, iValue);
    leftShooter.config_kD(0, dValue);
    leftShooter.config_kF(0, fValue);
    leftShooter.configAllowableClosedloopError(0, 0);
    leftShooter.configMaxIntegralAccumulator(0, maxIntegralAccumulator);
    leftShooter.configClosedLoopPeriod(0, PIDLoopRate);
  }

  //set each motor to desired speed using percent
  public void shoot (double bottomSpeed, double rightSpeed, double leftSpeed)
  {
    bottomShooter.set(ControlMode.PercentOutput, -1*bottomSpeed);
    rightShooter.set(ControlMode.PercentOutput, rightSpeed);
    leftShooter.set(ControlMode.PercentOutput, -leftSpeed);
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

  public void shootAtVelocity()
  {
    double height = getVertical();

    double velocity = 0;

    // if (height > 21)
    // {
    //   velocity = getShootVelocityMid();
    //   System.out.println("Shooting Mid");
    // }
    // else
    // {
    //   velocity = getShootVelocityFar();
    //   System.out.println("Shooting Far");
    // }

    velocity = getShootVelocityMid();

    velocity = velocity + getAngledSpeedDecrease();

    System.out.println(velocity);

    SmartDashboard.putNumber("Current Found Shooting Height", getVertical());
    SmartDashboard.putNumber("Current Velocity Target", velocity);
    SmartDashboard.putNumber("Current Percentage Target", velocity / 17500);
    spinToSpeed(velocity / 17500);
  }

  //formula for shooting speed with velocity and height
  public double getShootVelocityMid ()
  {
    double height = getVertical();

    // double velocity = 30 * Math.pow((height - 20), 2);
    // velocity = velocity + 9000;

    double velocity = 8 * Math.pow((height - 30), 2);
    velocity = velocity + Math.abs(height * 10);
    velocity = velocity + 3650;

    //double velocity = 6350;

    // System.out.println(velocity);

    // velocity = 17500;

    return velocity;
  }

  public double getShootVelocityFar ()
  {
    double height = getVertical();

    double velocity = 15 * Math.pow((height - 18), 2);
    velocity = velocity + Math.abs(height * 15);
    velocity = velocity + 7650;

    return velocity;
  }

  public double getAngledSpeedDecrease ()
  {
    double horizontal = getHorizontal();

    double offset = 18500 / horizontal;

    if (horizontal < 66)
    {
      offset = offset * 1.25;
    }

    return -offset;
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

  public void enableLimelight()
  {
    limelightTable.getEntry("ledMode").forceSetNumber(3);
  }

  public void disableLimelight()
  {
    limelightTable.getEntry("ledMode").forceSetNumber(1);
  }

  //set the shooter motors to desired speed using velocity
  public void spinToSpeed (double spinSpeed)
  {
    rightShooter.set(ControlMode.PercentOutput, spinSpeed * 1); // encoder ticks per 100ms
    bottomShooter.set(ControlMode.PercentOutput, -1 * spinSpeed * bottomSpin);
    leftShooter.set(ControlMode.PercentOutput, spinSpeed *  -1);
    // SmartDashboard.putNumber("Current Velocity", rightShooterSensor.getIntegratedSensorVelocity());

    // SmartDashboard.putNumber("right Output", rightShooter.getMotorOutputPercent());
    // SmartDashboard.putNumber("bottom Output", bottomShooter.getMotorOutputPercent());
    // SmartDashboard.putNumber("left Output", leftShooter.getMotorOutputPercent());
  }

  public void changeConfig()
  {
    pValue = sbpValue.getDouble(pValue);
    iValue = sbiValue.getDouble(iValue);
    dValue = sbdValue.getDouble(dValue);
    fValue = sbfValue.getDouble(fValue);
    maxIntegralAccumulator = (int)Math.floor(sbmaxIntegralAccumulator.getNumber(maxIntegralAccumulator).doubleValue());
    PIDLoopRate = sbPIDLoopRate.getNumber(PIDLoopRate).intValue() ;
    bottomSpin = sbbottomSpin.getDouble(bottomSpin);


    rightShooter.config_kP(0, SmartDashboard.getNumber("P Value Right", 0));
    rightShooter.config_kI(0, SmartDashboard.getNumber("I Value Right", 0));
    rightShooter.config_kD(0, SmartDashboard.getNumber("D Value Right", 0));
    rightShooter.config_kF(0, SmartDashboard.getNumber("F Value Right", 0));

    leftShooter.config_kP(0, SmartDashboard.getNumber("P Value Right", 0));
    leftShooter.config_kI(0, SmartDashboard.getNumber("I Value Right", 0));
    leftShooter.config_kD(0, SmartDashboard.getNumber("D Value Right", 0));
    leftShooter.config_kF(0, SmartDashboard.getNumber("F Value Right", 0));

    bottomShooter.config_kP(0, SmartDashboard.getNumber("P Value Right", 0));
    bottomShooter.config_kI(0, SmartDashboard.getNumber("I Value Right", 0));
    bottomShooter.config_kD(0, SmartDashboard.getNumber("D Value Right", 0));
    bottomShooter.config_kF(0, SmartDashboard.getNumber("F Value Right", 0));
  }

  @Override
  public void periodic() 
  {
    
  }
}