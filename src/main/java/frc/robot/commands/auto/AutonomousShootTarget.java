/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class AutonomousShootTarget extends CommandBase {
  
  ShooterSubsystem shooterSubsystem;
  StorageSubsystem storageSubsystem;

  //creates a new timer for intake
  Timer timer = new Timer();

  public AutonomousShootTarget(ShooterSubsystem subsystem, StorageSubsystem storageSubsystem) 
  {
    shooterSubsystem = subsystem;
    this.storageSubsystem = storageSubsystem;
    addRequirements(shooterSubsystem, storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //starts the timer
    timer.start();

    shooterSubsystem.shootAtVelocity();

    shooterSubsystem.disableLimelight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    if (timer.get() > 0.5)
    {
      //if timer reads half a second run the storage system
      storageSubsystem.runStorage(0.4);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    //when over set shooter and storage to 0
    storageSubsystem.runStorage(0);
    shooterSubsystem.shoot(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //finish early if the timer hits 10 seconds
    return timer.get() > 4;
  }
}
