package frc.robot.swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SpeedController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import frc.robot.Constants;

public class SwerveWheel {
    private PIDController rotationController;
    private AnalogPotentiometer potentiometer;
    private BaseMotorController driveMotor;
    private SpeedController twistMotor;
    private double offset;
    private boolean enabled = true;
    private double currentAngle;

    public SwerveWheel(PIDController rotationController, AnalogPotentiometer potentiometer, SpeedController twistMotor, BaseMotorController driveMotor, double offset) {
        System.out.println("wheel Initialized");
        this.potentiometer = potentiometer;
        this.rotationController = rotationController;
        this.driveMotor = driveMotor;
        this.twistMotor = twistMotor;
        this.offset = offset;
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

        if (this.enabled) {
            twistMotor.set(rotationController.calculate(this.potentiometer.get(), this.currentAngle));
        }
        else {
            twistMotor.set(0);
        }

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
    
            twistMotor.set(rotationController.calculate(this.potentiometer.get(), setpoint));
        }
        else {
            twistMotor.set(0);
        }

    }

    public void disableRotation() {
        this.enabled = false;
    }

    public void enableRotation() {
        this.enabled = true;
    }
}
