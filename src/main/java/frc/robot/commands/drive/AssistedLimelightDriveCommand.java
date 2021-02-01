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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
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

  // create limelight timer object
  Timer limeTime = new Timer();

  public AssistedLimelightDriveCommand(DriveSubsystem subsystem) {
    
    driveSubsystem = subsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //run the init method on drive subsystem......should not have to do this......
    this.driveSubsystem.init();
    this.driveSubsystem.enable();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double directionX = driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickXAxis);
    double directionY = driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickYAxis);
    double rotation = 0;
    boolean slowMode = driverController.getBumper(Hand.kLeft);
    
    //if pressing the right trigger
    if (driverController.getRawAxis(Constants.DriverControl.driverControllerRightTriggerAxis) > 0.2)
    {
      //limelightTable.getEntry("ledMode").forceSetNumber(1); // set Limelight LED Mode to OFF
      //nice
      SmartDashboard.putNumber("LimeLight Timer", limeTime.get()); // send the current value of the limelight timer to the dashboard
      if (limeTime.get() == 0 ){
        System.out.println("Starting Limelight Timer and turning LED On");
        limeTime.start(); // start the limelight LED timer
        limelightTable.getEntry("ledMode").forceSetNumber(3); // set Limelight LED Mode to ON
      }
      else if (limeTime.get() >= 5.75 && limelightTable.getEntry("ledMode").getDouble(0) == 1){ // if timer is over 5.75 and LED is off
        System.out.println("Limelight timer has reached 5.75 seconds and will now reset");
        limeTime.stop(); // stop the limelight timer
        limeTime.reset(); // reset the limelight timer to 0
      }
      else if (limeTime.get() >= 5 && limelightTable.getEntry("ledMode").getDouble(0) == 3){ // if timer is over 5 seconds and LED is on
        System.out.println("Limelight timer has reached 5 seconds and the LED will turn off");
        limelightTable.getEntry("ledMode").forceSetNumber(1); //Set the LED to off
      }
      else if (limeTime.get() < 5 && limelightTable.getEntry("ledMode").getDouble(0) == 1) // if timer is less than 5 and LED is off
      {
        System.out.println("Limelight timer is under 5 seconds, turning the LED on");
        limelightTable.getEntry("ledMode").forceSetNumber(3); // Set the LED to ON
      }

      //find the center of target
      horizontalEntry = limelightTable.getEntry("tx");
      horizontal = horizontalEntry.getDouble(0);
      horizontal = horizontal - 2.9/* + limelightTable.getEntry("thor").getDouble(0) / 15*/;
      
      rotation = horizontal / 23.0;
      rotation = rotation - rotation * 0.55;
      if (rotation > 0.2 && limelightTable.getEntry("ledMode").getDouble(0) == 3)
      {
        rotation = 0.2;
        SmartDashboard.putBoolean("Drive Aligned", false); // Dashboard drive align off
      }
      else if (rotation < -0.2 && limelightTable.getEntry("ledMode").getDouble(0) == 3)
      {
        rotation = -0.2;
        SmartDashboard.putBoolean("Drive Aligned", false); // Dashboard drive align off
      }
      
      if (Math.abs(rotation) <= 0.015 && limelightTable.getEntry("ledMode").getDouble(0) == 3)
      {
        // this will be run when once the robot has aligned it self with the target
        rotation = 0;
        SmartDashboard.putBoolean("Drive Aligned", true); // Dashboard drive align on
      }
      if (Math.abs(rotation) <= 0.015 || limelightTable.getEntry("ledMode").getDouble(0) == 1)
      {
        // this will be run when once the robot has aligned it self with the target
        System.out.println("Pausing rotation alignment while LED is off...");
        rotation = 0;
        //SmartDashboard.putBoolean("Drive Aligned", true); // Dashboard drive align on
      }
      SmartDashboard.putNumber("Align Rotation", rotation);
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
    } // End of if trigger pressed
    else
    {
      if (limelightTable.getEntry("ledMode").getDouble(0) == 3 || limeTime.get() >= 5) // If limelight led is set on or timer is not 0
      {
        limeTime.stop();
        limeTime.reset();
        limelightTable.getEntry("ledMode").forceSetDouble(1);
        SmartDashboard.putBoolean("Drive Aligned", false);  // Shuffleboard drive align off
        SmartDashboard.putNumber("LimeLight Timer", 0); // set the limelight timer to 0 when exiting alignment sequence
        System.out.println("Targeting Mode Exited, turning LED off");
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
