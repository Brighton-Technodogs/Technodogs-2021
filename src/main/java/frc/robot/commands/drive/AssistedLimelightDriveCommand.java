/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AssistedLimelightDriveCommand extends CommandBase {

  DriveSubsystem driveSubsystem;

  XboxController driverController = new XboxController(Constants.DriverControl.driverControllerPort);

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  NetworkTableEntry horizontalEntry;
  double horizontal;

  public AssistedLimelightDriveCommand(DriveSubsystem subsystem) {
    
    driveSubsystem = subsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    this.driveSubsystem.init();
    this.driveSubsystem.enable();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double directionX = driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickXAxis);
    double directionY = driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickYAxis);
    double rotation = 0;
    
    if (driverController.getBButton())
    {
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

      driveSubsystem.CircleDrive(-rotation);
    }
    else
    {
      rotation = driverController.getRawAxis(Constants.DriverControl.driverControllerRightStickXAxis);
      
      SmartDashboard.putNumber("X Box X-Axis", directionX);
      SmartDashboard.putNumber("X Box Y-Axis", directionY);
      SmartDashboard.putNumber("X Box Rotation", rotation);

      this.driveSubsystem.drive(directionX, directionY, rotation, false, true, false);
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
