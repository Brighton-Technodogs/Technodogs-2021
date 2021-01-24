/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import com.fasterxml.jackson.databind.ser.std.TimeZoneSerializer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class AutonomousLongShootCommand extends CommandBase {
  
  DriveSubsystem driveSubsystem;
  ShooterSubsystem shooterSubsystem;
  StorageSubsystem storageSubsystem;

  Timer timer = new Timer();

  double shootingOffset;
  
  public AutonomousLongShootCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem)
  {
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.storageSubsystem = storageSubsystem;

    addRequirements(this.driveSubsystem, this.shooterSubsystem, this.storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    driveSubsystem.enable();
    driveSubsystem.init();

    timer.start();

    double shootingHeight = shooterSubsystem.getVertical();

    shootingHeight = shootingHeight - 18;

    shootingOffset = shootingHeight / 10;

    shootingOffset = shootingOffset * 0.1;

    shootingOffset = shootingOffset - 0.015;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    driveSubsystem.xMode();

    storageSubsystem.runStorage(0.5);

    shooterSubsystem.shoot(0.55, 0.36 - shootingOffset, 0.36 - shootingOffset);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    shooterSubsystem.shoot(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return timer.get() > 5;
  }
}
