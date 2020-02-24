/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeWithSensorCommand extends CommandBase {
  
  IntakeSubsystem intakeSubsystem;

  private final XboxController operatrorController = new XboxController(Constants.OperatorControl.operatorControllerPort);

  private final DigitalInput intakeContrastSensor = new DigitalInput(Constants.Sensors.intakeContrastSensorDIO);

  private Timer timer = new Timer();
  private boolean firstContact = true;
  private double stopTime = 0.75;

  public RunIntakeWithSensorCommand(IntakeSubsystem subsystem) 
  {
    intakeSubsystem = subsystem;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    firstContact = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    //get contrast sensor value, false == seen, true == not seen. the ! flips this value for easier use
    boolean currentContrast = !intakeContrastSensor.get();

    //System.out.println(currentContrast);

    //run the intake at controller set speed

    double intakeSpeed = operatrorController.getRawAxis(Constants.OperatorControl.operatorLeftTrigger);

    intakeSubsystem.runIntake(intakeSpeed);

    if (currentContrast == true)
    {
      if (firstContact)
      {
        //first contact resets the timer
        timer.reset();
        timer.start();
        firstContact = false;
      }
      else if (timer.get() >= stopTime)
      {
        //if seeing the same objet for too long stops the motor
        intakeSubsystem.runStorage(0);
      }
      else
      {
        intakeSubsystem.runStorage(0.5);
      }
    }
    else
    {
      //resets all information when not seeing, and sets storage motor to 0
      firstContact = true;
      intakeSubsystem.runStorage(0);
      timer.stop();
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
