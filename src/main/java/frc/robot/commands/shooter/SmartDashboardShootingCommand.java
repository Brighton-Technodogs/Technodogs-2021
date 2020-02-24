/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class SmartDashboardShootingCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;

  private final XboxController operatorController = new XboxController(Constants.OperatorControl.operatorControllerPort);

  public SmartDashboardShootingCommand(ShooterSubsystem subsystem) 
  {
    
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    SmartDashboard.putNumber("Shooting Speed", 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    SmartDashboard.putNumber("Array Index", (int)shooterSubsystem.getDistance(shooterSubsystem.getArea()));

    double shootSpeed = SmartDashboard.getNumber("Shooting Speed", 0);

    if (operatorController.getRawAxis(Constants.OperatorControl.operatorRightTrigger) > 0.2)
    {
      shooterSubsystem.shoot(shootSpeed, shootSpeed, shootSpeed);
    }

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
