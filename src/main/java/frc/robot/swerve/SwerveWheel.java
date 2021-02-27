package frc.robot.swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/*

The SwerveWheel is the lowest layer (closest to the hardware). It contains
the motors (one for drive, one for twist) as well as the analog (absolute) 
encoder. It also contains a PIDController that is used to control the rotational
position of the wheel (twist). 

The way this is used, is that the all of the hardware is instansiated in
the DriveSubsystem, the SwerveWheel's are created, and then the SwerveWheel's
are passed into the SwerveDrive.

The drive method is called, taking in a speed (percent) and a rotation (degrees).
These are calculated using swerve math in SwerveDrive. This is called
in normal driving scenarios

UpdateRotation uses the PID to calculate motor power needed to get the 
wheel to the correct rotation. This is called to set the wheels to certain
modes such as "X Mode" to avoid being pushed. UpdateRotation called with
no arguments turns the motors off. This prevents them from shaking when
the robot is not being driven/moved.

*/

public class SwerveWheel {
    private PIDController rotationController;
    private AnalogPotentiometer potentiometer;
    private BaseMotorController driveMotor;
    private VictorSPX twistMotor;
    private double offset;
    private boolean enabled = true;
    private String moduleID;

    public SwerveWheel(PIDController rotationController, AnalogPotentiometer potentiometer, VictorSPX twistMotor, BaseMotorController driveMotor, double offset, String moudleId) {
        // System.out.println("wheel Initialized");
        this.potentiometer = potentiometer;
        this.rotationController = rotationController;
        this.driveMotor = driveMotor;
        this.twistMotor = twistMotor;
        this.offset = offset;
        this.moduleID = moudleId;
    }

    /**
     * Drives a single swerve wheel.
     * 
     * @param newSpeed The speed at which to drive the drive wheel
     * @param newAngle The angle at which to position the drive wheel
     */
    public void drive(double newSpeed, double newAngle) {
        updateSpeed(newSpeed);
        updateRotation(newAngle);
    }

    /**
     * Update the speed at which to drive the wheel
     * 
     * @param newSpeed The speed at which to drive the drive wheel
     */
    public void updateSpeed(double newSpeed) {
        if(enabled) {
            driveMotor.set(ControlMode.PercentOutput, newSpeed);
        }
    }

    /**
     * Update the angle at which to position the drive wheel
     */
    public void updateRotation() {

        SmartDashboard.putNumber("Commanded Motor Speed " + this.moduleID, 0);
        twistMotor.set(ControlMode.PercentOutput, 0);
        return;
        // if (this.enabled) {
        //     twistMotor.set(rotationController.calculate(this.potentiometer.get(), this.currentAngle));
        // }
        // else {
        //     twistMotor.set(0); // Turn motor off if rotation is disabled.
        // }
        // Don't set the twist to 0 when disabled, just leave it set the way it is.
        // else {
        //     twistMotor.set(0);
        // }

    }

    /**
     * Update the angle at which to position the drive wheel
     * 
     * @param newAngle The angle at which to position the drive wheel
     */
    public void updateRotation(double newAngle) {
        // System.out.println("Updating Rotation. Current Encoder value = [" + this.potentiometer.get() + "] Setpoint = [" + newAngle + "]");

        if (this.enabled) {
            newAngle = newAngle + offset;
            double setpoint;
    
            if (newAngle < 0) {
                setpoint =  360 - (newAngle * -1);
            } else if (newAngle > 360) {
                setpoint = newAngle - 360;
            } else {
                setpoint = newAngle;
            }
    
            double motorOutput = rotationController.calculate(this.potentiometer.get(), setpoint);

            SmartDashboard.putNumber("Commanded Set Point " + this.moduleID, setpoint);
            SmartDashboard.putNumber("Commanded Motor Speed " + this.moduleID, motorOutput);
            SmartDashboard.putNumber("Position Error " + this.moduleID, rotationController.getPositionError());

            twistMotor.set(ControlMode.PercentOutput, motorOutput);
        }
        else {
            twistMotor.set(ControlMode.PercentOutput, 0); // Turn motor off if rotation is disabled.
        }
        // Don't set the twist to 0 when disabled, just leave it set the way it is.
        // else {
        //     twistMotor.set(0);
        // }

    }

    /**
     * Disables rotation by turning motors off.
     * */ 

    public void disableRotation() {
        this.enabled = false;
    }

    /**
     * Enables rotation by enabling PID control
     * */ 
    public void enableRotation() {
        this.enabled = true;
    }
}
