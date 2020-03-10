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
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AssistedLimelightDriveCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  boolean limeLightarmed = false;

  XboxController driverController = new XboxController(Constants.DriverControl.driverControllerPort);
  XboxController operatorController = new XboxController(Constants.OperatorControl.operatorControllerPort);

  //initialize link to limelight network table
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
    //run the init method on drive subsystem......should not have to do this......
    // this.driveSubsystem.init();
    // this.driveSubsystem.enable();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double directionX = driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickXAxis);
    double directionY = driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickYAxis);
    double rotation = 0;
    boolean slowMode = driverController.getBumper(Hand.kLeft);
    
    //if pressing the B - Button
    if (driverController.getRawAxis(Constants.DriverControl.driverControllerRightTriggerAxis) > 0.2)
    {

      limelightTable.getEntry("ledMode").forceSetNumber(3);

      //find the center of target
      horizontalEntry = limelightTable.getEntry("tx");
      horizontal = horizontalEntry.getDouble(0);
      horizontal = horizontal - 2.9/* + limelightTable.getEntry("thor").getDouble(0) / 15*/;
      
      
      rotation = horizontal / 23.0;
      rotation = rotation - rotation * 0.55;
      if (rotation > 0.2)
      {
        rotation = 0.2;
      }
      else if (rotation < -0.2)
      {
        rotation = -0.2;
      }
      
      if (Math.abs(rotation) <= 0.015)
      {
        rotation = 0;
      }

      //Not needed from Jacob T. Save for later if desired
      // if (operatorController.getRawAxis(Constants.OperatorControl.operatorRightTrigger) > 0.2)
      // {
      //   rotation = 0;
      // }

      double controllerAssist = driverController.getRawAxis(Constants.DriverControl.driverControllerRightStickXAxis) / 15;

      //spin to center on target
      
      if (driverController.getXButton())
      {
        driveSubsystem.xMode();
      }
      else if(driverController.getRawAxis(Constants.DriverControl.driverControllerLeftTriggerAxis) > 0.2)
      {
        driveSubsystem.CircleDrive(-controllerAssist * 1.5);
      }
      else
      {
        driveSubsystem.CircleDrive(-rotation - controllerAssist);
      }
    }
    else
    {

      if (limelightTable.getEntry("ledMode").getDouble(0) == 3)
      {
        limelightTable.getEntry("ledMode").forceSetDouble(1);
      }

      rotation = driverController.getRawAxis(Constants.DriverControl.driverControllerRightStickXAxis);
      
      // SmartDashboard.putNumber("X Box X-Axis", directionX);
      // SmartDashboard.putNumber("X Box Y-Axis", directionY);
      // SmartDashboard.putNumber("X Box Rotation", rotation);

      //drive normally with joysticks
      this.driveSubsystem.drive(directionX, directionY, rotation, false, slowMode, false);
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
