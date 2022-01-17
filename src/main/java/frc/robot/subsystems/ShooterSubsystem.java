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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controllers.LazyTalonFX;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.LimelightPositionCalc;
import frc.lib.sensors.Limelight.LimeLedMode;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  //create the shooter objects from constants can ID
  private LazyTalonFX bottomShooter = new LazyTalonFX(Constants.ShooterSubsystem.bottomShooterFalconCan);
  private LazyTalonFX rightShooter = new LazyTalonFX(Constants.ShooterSubsystem.rightShooterFalconCan);
  private LazyTalonFX leftShooter = new LazyTalonFX(Constants.ShooterSubsystem.leftShooterFalconCan);
  
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

  Limelight limelight = new Limelight();
  // numbers: Camera Height, Camera Angle, Target Height
  LimelightPositionCalc limePosCalc = new LimelightPositionCalc(limelight, 2.5, 45, 8); // TODO: figure out these numbers


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

private NetworkTableEntry calcDist = subsystemShuffleboardTab.add("Calc Dist", 0)
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

  public void shootAtVelocity()
  {
    double distance = limePosCalc.calculate();
    calcDist.setDouble(distance);
    double targetVelocity = getTargetVelocity(distance);
    spinToSpeed(targetVelocity);
    SmartDashboard.putNumber("Current Found Distance", distance);
    SmartDashboard.putNumber("Current Velocity Target", targetVelocity);
  }

  public double getTargetVelocity(double distance) {
    double vel = (distance * 1000);
    // TODO: Make this equation
    return vel; // testing speed
  }

  //set the shooter motors to desired speed using velocity
  public void spinToSpeed (double spinSpeed)
  {
    rightShooter.set(ControlMode.Velocity, spinSpeed * 1); // encoder ticks per 100ms
    bottomShooter.set(ControlMode.Velocity, -1 * spinSpeed * bottomSpin);
    leftShooter.set(ControlMode.Velocity, spinSpeed *  -1);
    // SmartDashboard.putNumber("Current Velocity", rightShooterSensor.getIntegratedSensorVelocity());

    shooterRV.setDouble(spinSpeed);
    shooterBV.setDouble(spinSpeed*bottomSpin);
    shooterLV.setDouble(spinSpeed);

    gshooterRV.setDouble(spinSpeed);
    gshooterBV.setDouble(spinSpeed*bottomSpin);
    gshooterLV.setDouble(spinSpeed);
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
    if (limelight.getLED() == LimeLedMode.ON) {
      SmartDashboard.putNumber("Limelight VO", limelight.getVerticalOffset());
      SmartDashboard.putNumber("Limelight HO", limelight.getHorizontalOffset());
      SmartDashboard.putNumber("Limelight Height", limelight.getTargetHeight());
      SmartDashboard.putNumber("Limelight Width", limelight.getTargetWidth());
      SmartDashboard.putNumber("Limelight Area", limelight.getTargetArea());

      llXCoord.setDouble(limelight.getVerticalOffset());
      llYCoord.setDouble(limelight.getVerticalOffset());
      llHoriz.setDouble(limelight.getTargetWidth());
      llVert.setDouble(limelight.getTargetHeight());
      llArea.setDouble(limelight.getTargetArea());
    }

    shooterLS.setDouble(leftShooterSensor.getIntegratedSensorVelocity());
    shooterBS.setDouble(bottomShooterSensor.getIntegratedSensorVelocity());
    shooterRS.setDouble(rightShooterSensor.getIntegratedSensorVelocity());

    gshooterLS.setDouble(leftShooterSensor.getIntegratedSensorVelocity());
    gshooterBS.setDouble(bottomShooterSensor.getIntegratedSensorVelocity());
    gshooterRS.setDouble(rightShooterSensor.getIntegratedSensorVelocity());
  }
}