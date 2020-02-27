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
  IntakeSubsystem intakeSubsystem;
  StorageSubsystem storageSubsystem;

  //creates a new timer for intake
  Timer timer = new Timer();

  public AutonomousShootTarget(ShooterSubsystem subsystem, IntakeSubsystem intakeSubsystem, StorageSubsystem storageSubsystem) 
  {
    shooterSubsystem = subsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.storageSubsystem = storageSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem, storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //starts the timer
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //array index of distance to target based on area
    int index = (int)shooterSubsystem.getDistance(shooterSubsystem.getArea());
    
    //set speed to 0
    double autoShooterSpeed = 0;
    
    if (index < shooterSubsystem.speeds.length - 1)
    {
      //if index is in speeds array length set correct speed
      autoShooterSpeed = shooterSubsystem.speeds[index];
    }
    else
    {
      //if not in length then set to 0.65 | will be changed at later date
      autoShooterSpeed = 0.65;
    }

    //shoot the shooter at required speed
    shooterSubsystem.shoot(autoShooterSpeed, autoShooterSpeed, autoShooterSpeed);

    /*if (timer.get() > 0.5)
    {
      //if timer reads half a second run the storage system
      intakeSubsystem.runStorage(0.5);
    }*/

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
    return timer.get() > 10;
  }
}
