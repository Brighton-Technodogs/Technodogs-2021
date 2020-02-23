/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousRotateToTarget extends CommandBase {
  
  DriveSubsystem driveSubsystem;

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  NetworkTableEntry horizontalEntry;
  double horizontal;
  double rotation = 0.1;

  public AutonomousRotateToTarget(DriveSubsystem subsystem) 
  {
    driveSubsystem = subsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (limelightTable.getEntry("tv").getDouble(0) == 0)
    {
      driveSubsystem.CircleDrive(0.15);
    }
    else
    {
      driveSubsystem.CircleDrive(0);

      horizontalEntry = limelightTable.getEntry("tx");
      horizontal = horizontalEntry.getDouble(0);
      rotation = horizontal / 23.0;
      rotation = rotation - rotation * 0.35;
      if (rotation > 0.15)
      {
        rotation = 0.15;
      }
      else if (rotation < -0.15)
      {
        rotation = -0.15;
      }
    
      if (Math.abs(rotation) <= 0.1)
      {
        rotation = 0;
      }

      //System.out.println(rotation);

      driveSubsystem.CircleDrive(-rotation);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    driveSubsystem.CircleDrive(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotation == 0;
  }
}
