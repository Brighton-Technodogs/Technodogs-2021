package frc.robot.swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SpeedController;

import frc.robot.Constants;

public class SwerveWheel {
    private PIDController rotation;
    private AnalogPotentiometer potentiometer;
    private SpeedController driveMotor;
    private SpeedController twistMotor;
    private double offset;
    private boolean enabled = true;
    private double currentAngle;

    public SwerveWheel(PIDController rotation, AnalogPotentiometer potentiometer, SpeedController twistMotor, SpeedController driveMotor, double offset) {
        System.out.println("wheel Initialized");
        this.potentiometer = potentiometer;
        this.rotation = rotation;
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
            driveMotor.set(newSpeed);
        }
    }

    /**
     * Update the angle at which to position the drive wheel
     */
    public void updateRotation() {

        System.out.println(this.potentiometer.get());
        rotation.

        if (this.enabled) {
            twistMotor.set(rotation.calculate(this.potentiometer.get(), this.currentAngle));
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
        this.currentAngle = newAngle;

        System.out.println(this.potentiometer.get());

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
    
            twistMotor.set(rotation.calculate(this.potentiometer.get(), setpoint));
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
