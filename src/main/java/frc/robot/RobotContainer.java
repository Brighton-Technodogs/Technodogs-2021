/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.commands.shooter.ChangeConfigCommand;
import frc.robot.commands.shooter.QuickFireCommand;
import frc.robot.commands.storage.ReverseStorageCommand;
import frc.robot.commands.storage.RunStorageCommand;
import frc.robot.commands.storage.RunStorageWithSensorCommand;
import frc.robot.commands.drive.AssistedLimelightDriveCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.auto.AutonomousSequentialCommandGroup;

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Subsystems
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  // private final StorageSubsystem storageSubsystem = new StorageSubsystem();

  //Drive Commands
  private final AssistedLimelightDriveCommand assistedLimelightDriveCommand = new AssistedLimelightDriveCommand(driveSubsystem);

  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  //Shooter Commands
  // private final AutoShootCommand autoShootCommand = new AutoShootCommand(shooterSubsystem);
  // private final QuickFireCommand quickFireCommand = new QuickFireCommand(shooterSubsystem);
  // private final ChangeConfigCommand changeConfigCommand = new ChangeConfigCommand(shooterSubsystem);

  //Storage Commands
  // private final RunStorageCommand runStorageCommand = new RunStorageCommand(storageSubsystem);
  // private final ReverseStorageCommand reverseStorageCommand = new ReverseStorageCommand(storageSubsystem);
  // private final RunStorageWithSensorCommand runStorageWithSensorCommand = new RunStorageWithSensorCommand(storageSubsystem);

  //Auto Commands
  // private final AutonomousSequentialCommandGroup autonomousSequentialCommandGroup = new AutonomousSequentialCommandGroup(driveSubsystem, shooterSubsystem, intakeSubsystem);

  //Operator Contoller and Buttons
  private final XboxController operatorController = new XboxController(1);
  private final JoystickButton operatorBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
  private final JoystickButton operatorAButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
  private final JoystickButton operatorXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
  private final JoystickButton operatorYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);

  
  public RobotContainer() 
  {
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(driveCommand);

    // shooterSubsystem.setDefaultCommand(autoShootCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("Configuring Button Bindings");

    // operatorBButton.whenHeld(quickFireCommand);

    // operatorAButton.whenHeld(runStorageWithSensorCommand);

    // operatorXButton.whenHeld(reverseStorageCommand);

    // operatorYButton.whenPressed(changeConfigCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutoCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.DriveSubsystem.kMaxSpeedMetersPerSecond,
        Constants.DriveSubsystem.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveSubsystem.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(
            0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config
    );

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        driveSubsystem::getPose, //Functional interface to feed supplier
        Constants.DriveSubsystem.kDriveKinematics,

        //Position controllers
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(3,3)),

        driveSubsystem::setModuleStates,

        driveSubsystem

    );

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, false));
  }
}
