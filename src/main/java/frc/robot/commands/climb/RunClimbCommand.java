/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class RunClimbCommand extends CommandBase {
  
  ClimbSubsystem climbSubsystem;

  XboxController operatorController = new XboxController(Constants.OperatorControl.operatorControllerPort);

  boolean inverseControl = true;

  public RunClimbCommand(ClimbSubsystem climb) 
  {
    climbSubsystem = climb;

    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    double climbSpeed = operatorController.getRawAxis(Constants.OperatorControl.operatorControllerLeftStickYAxis);

    climbSpeed *= inverseControl ? -1 : 1;


    // if (climbSpeed < -0.85)
    // {
    //   climbSpeed = -1;
    // }
    // else
    // {
    //   climbSpeed = climbSpeed * 0.5;
    // }

    if (operatorController.getYButton())
    {
      climbSpeed = climbSpeed * 0.7;
    }
    else
    {
      climbSpeed = climbSpeed * 0.3;
    }
    

    climbSubsystem.runClimb(climbSpeed);

    double winchSpeed = operatorController.getRawAxis(Constants.OperatorControl.operatorControllerRightStickYAxis);

    climbSubsystem.runWinch(winchSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
