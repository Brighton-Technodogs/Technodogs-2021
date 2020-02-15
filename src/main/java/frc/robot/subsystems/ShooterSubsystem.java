/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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
    bottomShooter.set(ControlMode.PercentOutput, bottomSpeed);
    rightShooter.set(ControlMode.PercentOutput, rightSpeed);
    leftShooter.set(ControlMode.PercentOutput, leftSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
