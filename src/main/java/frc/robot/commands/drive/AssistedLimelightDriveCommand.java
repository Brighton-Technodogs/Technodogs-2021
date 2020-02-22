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
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double directionX = driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickXAxis);
    double directionY = driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickYAxis);
    double rotation = 0;
    
    if (driverController.getRawAxis(Constants.DriverControl.driverControllerRightTriggerAxis) >= 0.2)
    {
      horizontalEntry = limelightTable.getEntry("tx");
      horizontal = horizontalEntry.getDouble(0);
      rotation = horizontal / 23.0;
      rotation = rotation * 0.6;
    }
    else
    {
      rotation = driverController.getRawAxis(Constants.DriverControl.driverControllerRightStickXAxis);
    }

    driveSubsystem.drive(directionX, directionY, rotation, false, false, false);

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
