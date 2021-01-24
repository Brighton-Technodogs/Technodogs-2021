/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class GenericMoveAutonomousCommand extends CommandBase {
  
  DriveSubsystem driveSubsystem;

  Timer timer = new Timer();

  double runTime;
  double runSpeed;
  double runAngle;

  public GenericMoveAutonomousCommand(DriveSubsystem drive, double time, double speed, double angle) 
  {
    driveSubsystem = drive;

    runTime = time;
    runSpeed = speed;
    runAngle = angle;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    driveSubsystem.init();
    driveSubsystem.enable();

    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    if (timer.get() < 0.5)
    {
      driveSubsystem.driveSimple(0, runAngle);
    }
    else
    {
      driveSubsystem.driveSimple(runSpeed, runAngle);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    driveSubsystem.driveSimple(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > runTime;
  }
}
