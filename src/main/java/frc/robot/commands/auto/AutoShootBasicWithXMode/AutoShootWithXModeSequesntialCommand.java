/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.AutoShootBasicWithXMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutonomousMoveForward;
import frc.robot.commands.auto.AutonomousRotateToTarget;
import frc.robot.commands.auto.AutonomousShootTarget;
import frc.robot.commands.auto.AutonomousXModeCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.ResetIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootWithXModeSequesntialCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutoShootWithXModeSequesntialCommand.
   */
  public AutoShootWithXModeSequesntialCommand(DriveSubsystem drive, ShooterSubsystem shoot, StorageSubsystem storage, IntakeSubsystem intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DeployIntakeCommand(intake), new AutonomousRotateToTarget(drive), new AutonomousXModeCommand(drive), new AutonomousShootTarget(shoot, storage, drive), new AutonomousMoveForward(drive), new ResetIntakeCommand(intake));
  }
}
