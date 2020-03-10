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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousRotateToTarget extends CommandBase {
  
  DriveSubsystem driveSubsystem;

  //initialize the network table link
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  //variables for network table x coordinate to rotation
  NetworkTableEntry horizontalEntry;
  double horizontal;
  double rotation = 0.1;

  Timer timer = new Timer();

  public AutonomousRotateToTarget(DriveSubsystem subsystem) 
  {
    //grab our drive subsystem and add the requirement
    driveSubsystem = subsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    limelightTable.getEntry("ledMode").forceSetNumber(3);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    limelightTable.getEntry("ledMode").forceSetNumber(3);

    //test if the target can be seen, if no spin
    if (limelightTable.getEntry("tv").getDouble(0) == 0)
    {
      // driveSubsystem.CircleDrive(-0.15);
    }
    //if it can see the target
    else
    {
      //stop old spin
      //driveSubsystem.CircleDrive(0);

      //get position of target on camera
      horizontalEntry = limelightTable.getEntry("tx");
      horizontal = horizontalEntry.getDouble(0);
      horizontal = horizontal - 1.5;
      rotation = horizontal / 23.0;
      //rotation = rotation - rotation * 0.35;
      // System.out.println(rotation);
      //set a max to the rotation
      if (rotation > 0.1)
      {
        rotation = 0.1;
      }
      else if (rotation < -0.1)
      {
        rotation = -0.1;
      }
    
      if (Math.abs(rotation) < 0.07)
      {
        rotation = 0;
      }

      //System.out.println(rotation);

      //spin according to rotation value
      // driveSubsystem.CircleDrive(-rotation);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    //stop spinning when command is over
    // driveSubsystem.CircleDrive(0);
    driveSubsystem.driveSimple(0, 0);
    System.out.println("Im done");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //finish when target is in the centered
    return rotation == 0;
  }
}
