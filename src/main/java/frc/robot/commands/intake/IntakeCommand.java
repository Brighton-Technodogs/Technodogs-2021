/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  
  IntakeSubsystem intakeSubsystem;

  private final XboxController operatrorController = new XboxController(Constants.XboxAxixMapping.operatorControllerPort);
  
  /**
   * Creates a new IntakeCommand.
   */
  public IntakeCommand(IntakeSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    intakeSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    /*double speed = SmartDashboard.getNumber("Intake Speed", 0);

    intakeSubsystem.runStorage(speed);*/

    double intakeSpeed = operatrorController.getRawAxis(Constants.XboxAxixMapping.operatorLeftTrigger); //add controller support when Joey allows it

    if (intakeSpeed > 0.1)
    {
      intakeSubsystem.runStorage(1);
    }
    else
    {
      intakeSubsystem.runStorage(0);
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
