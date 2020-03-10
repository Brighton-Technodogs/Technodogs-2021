/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.storage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.StorageSubsystem;

public class RunStorageWithSensorCommand extends CommandBase {

  StorageSubsystem storageSubsystem;
  ;
  
  private final DigitalInput contrastSensor = new DigitalInput(Constants.Sensors.storageContrastSensorDIO);

  public RunStorageWithSensorCommand(StorageSubsystem subsystem)
  {
    storageSubsystem = subsystem;

    addRequirements(storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    //boolean canSee = !contrastSensor.get();

    // storageSubsystem.runStorage(canSee ? 0.45 : 0.65);
    storageSubsystem.runStorage(0.4);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    storageSubsystem.runStorage(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
