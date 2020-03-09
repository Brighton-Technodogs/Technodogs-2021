/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  
  private VictorSPX climbMotor = new VictorSPX(Constants.ClimbSubsystemConstants.climbMotor);
  private CANSparkMax winchMotor = new CANSparkMax(51, MotorType.kBrushless);


  public ClimbSubsystem() {

  }

  public void runClimb (double speed)
  {
    // if (speed > 0)
    // {
    //   speed = speed * 0.25;
    // }
    // else if(speed < -0.85)
    // {
    //   speed = -1;
    // }
    // else
    // {
    //   speed = speed * 0.75;
    // }

    // System.out.println(speed);

    climbMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runWinch (double speed)
  {
    if (speed > 0.2)
    {
      winchMotor.set(speed);
    }
    else if (speed < -0.75)
    {
      winchMotor.set(-0.2);
    }
    else
    {
      winchMotor.set(0);
    }
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
