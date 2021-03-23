/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class LongShootCommand extends CommandBase {
  
  ShooterSubsystem shooterSubsystem;

  double shootingOffset;

  private final XboxController operatorController = new XboxController(Constants.OperatorControl.operatorControllerPort);

  public LongShootCommand(ShooterSubsystem subsystem)
  {
    shooterSubsystem = subsystem;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    // double shootingOffset = shooterSubsystem.getVertical() - 21;

    // if (shootingOffset > 0)
    // {
    //   shootingOffset = 0.05;
    // }
    // else if (shootingOffset < 0)
    // {
    //   shootingOffset = -0.05;
    // }

    // double shootingHeight = shooterSubsystem.getVertical();

    // shootingHeight = shootingHeight - 18;

    // shootingOffset = shootingHeight / 10;

    // shootingOffset = shootingOffset * 0.1;

    // shootingOffset = shootingOffset - 0.015;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double trigger_value = operatorController.getRawAxis(Constants.OperatorControl.operatorRightTrigger);

    if (trigger_value >= 0.1) 
    {
      shooterSubsystem.shoot(trigger_value, trigger_value, trigger_value);
    } else
    {
      shooterSubsystem.shoot(0, 0, 0);
    }

    // shooterSubsystem.shoot(0.55, 0.38 - shootingOffset, 0.38 - shootingOffset);

    // shooterSubsystem.shoot(0.6 + shootingOffset, 0.35 + shootingOffset, 0.35 + shootingOffset);

    // shooterSubsystem.shoot(1, 1, 1);

    //closer shooting, height of 24
    // shooterSubsystem.shoot(0.55, 0.30, 0.30); //0.35 needs changing saturday comp

    //farthest shot needed, vertical 20
    // shooterSubsystem.shoot(0.55, 0.4, 0.4);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    shooterSubsystem.shoot(0, 0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
