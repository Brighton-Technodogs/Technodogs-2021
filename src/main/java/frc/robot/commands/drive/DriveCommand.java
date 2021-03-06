// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 *
 */
public class DriveCommand extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;

    XboxController m_driverController = new XboxController(Constants.DriverControl.driverControllerPort);

    public DriveCommand(DriveSubsystem driveSubsystem) {
        System.out.println("Constructing DriveCommand");
        this.mDriveSubsystem = driveSubsystem;
        this.addRequirements(driveSubsystem);

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        System.out.println("Initializing DriveCommand");
        this.mDriveSubsystem.init();
        this.mDriveSubsystem.enable();
    }

    // Lidar lidarCrab = new Lidar(new DigitalInput(10));

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        double directionX = m_driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickXAxis);
        double directionY = m_driverController.getRawAxis(Constants.DriverControl.driverControllerLeftStickYAxis);
        double rotation = m_driverController.getRawAxis(Constants.DriverControl.driverControllerRightStickXAxis);
        
        this.mDriveSubsystem.drive(directionX, directionY, rotation, false, true, false);

        // Robot.driveSubsystem.drive(-Robot.oi.driverController.getLeftStickXValue(), -Robot.oi.driverController.getLeftStickYValue(),
        //             -Robot.oi.driverController.getRightStickXValue(), false,
        //             Robot.oi.driverController.getRightBumperPressed(), Robot.oi.driverController.getXButtonPressed());
    }

}
