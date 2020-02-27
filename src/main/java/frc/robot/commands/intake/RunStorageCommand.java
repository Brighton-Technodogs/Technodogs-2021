/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunStorageCommand extends CommandBase {
  
  IntakeSubsystem intakeSubsystem;

  private final XboxController operatorController = new XboxController(Constants.OperatorControl.operatorControllerPort);

  Timer timer = new Timer();

  public RunStorageCommand(IntakeSubsystem subsystem) 
  {
    intakeSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // double storageSpeed = operatorController.getRawAxis(Constants.XboxAxixMapping.operatorLeftTrigger);

    // if (storageSpeed > 0.1)
    // {
    //   if (timer.get() >= 0.15)
    //   {
    //     timer.reset();
    //   }
    //   else if (timer.get() >= 0.1)
    //   {
    //     intakeSubsystem.runStorage(0.75);
    //   }
    //   else
    //   {
    //     intakeSubsystem.runStorage(1);
    //   }
    // }
    // else
    // {
    //   intakeSubsystem.runStorage(0);
    // }

    //intakeSubsystem.runStorage(0.75);

    //this if, else if, else tree runs the storage at interval speeds to make balls not hit eachother

      intakeSubsystem.runStorage(1);

    /*if (timer.get() >= 0.15)
    {
      timer.reset();
    }
    else if (timer.get() >= 0.05)
    {
      intakeSubsystem.runStorage(0.5);
    }
    else
    {
      intakeSubsystem.runStorage(0.75);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //when over resest motor to 0
    intakeSubsystem.runStorage(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
