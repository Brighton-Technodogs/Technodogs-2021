/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoBackwardsShooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutonomousDriveByGyroCommand;
import frc.robot.commands.auto.AutonomousLongShootCommand;
import frc.robot.commands.auto.AutonomousRotateToTarget;
import frc.robot.commands.auto.AutonomousRunIntakeCommand;
import frc.robot.commands.auto.AutonomousShootTarget;
import frc.robot.commands.auto.GenericMoveAutonomousCommand;
import frc.robot.commands.auto.WaitCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.ResetIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousBackwardsShootingSequentialCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousBackwardsShootingSequentialCommand.
   */
  public AutonomousBackwardsShootingSequentialCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, IntakeSubsystem intakeSubsystem)
  {
    //Generic Moving constructor: drive, time, speed, angle
    /*super
    (
      new DeployIntakeCommand(intakeSubsystem),
      new AutonomousRotateToTarget(driveSubsystem), 
      new AutonomousShootTarget(shooterSubsystem, storageSubsystem),
      // new ReturnToGyroCommand(driveSubsystem),
      new AutonomousRunIntakeCommand(intakeSubsystem),
      new GenericMoveAutonomousCommand(driveSubsystem, 2, -0.25, 0), 
      new WaitCommand(0.75), 
      new AutonomousRotateToTarget(driveSubsystem), 
      new AutonomousShootTarget(shooterSubsystem, storageSubsystem),
      new ResetIntakeCommand(intakeSubsystem)
    );*/
    
    super(
      new DeployIntakeCommand(intakeSubsystem),
      new WaitCommand(0.25),
      new AutonomousRunIntakeCommand(intakeSubsystem), 
      // new GenericMoveAutonomousCommand(driveSubsystem, 3.75, -0.15, 0), 
      new AutonomousDriveByGyroCommand(driveSubsystem, 0, -0.1, 2),
      new WaitCommand(1), 
      new AutonomousRotateToTarget(driveSubsystem), 
      // new AutonomousShootTarget(shooterSubsystem, storageSubsystem, driveSubsystem),
      new AutonomousLongShootCommand(driveSubsystem, shooterSubsystem, storageSubsystem),
      new ResetIntakeCommand(intakeSubsystem)
    );
  }
}