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

public class ShootCommand extends CommandBase {
  
  ShooterSubsystem shooterSubsystem;
  
  private final XboxController operatrorController = new XboxController(Constants.XboxAxixMapping.operatorControllerPort);

  /**
   * Creates a new ShootCommand.
   */
  public ShootCommand(ShooterSubsystem subsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double shootSpeed = operatrorController.getRawAxis(Constants.XboxAxixMapping.operatorRightTrigger);//driver input when Joey allows it

    if (shootSpeed > 0.1) 
    {
      shooterSubsystem.shoot(shootSpeed, shootSpeed, shootSpeed);
    }
    else
    {
      shooterSubsystem.shoot(0, 0, 0); //stop the motor without input
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
