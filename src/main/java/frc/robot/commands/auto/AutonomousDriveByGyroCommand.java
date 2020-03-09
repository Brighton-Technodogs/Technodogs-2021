/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDriveByGyroCommand extends CommandBase {
  
  DriveSubsystem driveSubsystem;

  Timer timer = new Timer();

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  double xSpeed;
  double ySpeed;
  double time;

  public AutonomousDriveByGyroCommand(DriveSubsystem driveSubsystem, double xSpeed, double ySpeed, double time) 
  {
    this.driveSubsystem = driveSubsystem;

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.time = time;

    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    driveSubsystem.enable();
    driveSubsystem.init();

    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    // double rotation = gyro.getAngle() / 180;

    double angle = gyro.getAngle();

    double rotation = -angle;

    driveSubsystem.drive(0, -0.1, rotation, false, false, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    driveSubsystem.driveSimple(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return timer.get() > time;
  }
}
