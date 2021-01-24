/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.ResetIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousSequentialCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousSequentialCommandGroup.
   */
  public AutonomousSequentialCommandGroup(DriveSubsystem drive, ShooterSubsystem shoot, StorageSubsystem storage, IntakeSubsystem intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DeployIntakeCommand(intake), new AutonomousRotateToTarget(drive), new AutonomousShootTarget(shoot, storage, drive), new AutonomousMoveForward(drive), new ResetIntakeCommand(intake));
  }
}
