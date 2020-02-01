package frc.robot.swerve;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

public class SwerveWheel {
    private PIDController rotation;
    private SpeedController speed;
    private double offset;
    private boolean enabled = true;

    public SwerveWheel(PIDController rotation, SpeedController speed, double offset) {
        System.out.println("wheel Initialized");
        this.rotation = rotation;
        this.speed = speed;
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
            speed.set(newSpeed);
        }
    }

    /**
     * Update the angle at which to position the drive wheel
     * 
     * @param newAngle The angle at which to position the drive wheel
     */
    public void updateRotation(double newAngle) {
        newAngle = newAngle + offset;

        if (newAngle < 0) {
            rotation.setSetpoint(360 - (newAngle * -1));
        } else if (newAngle > 360) {
            rotation.setSetpoint(newAngle - 360);
        } else {
            rotation.setSetpoint(newAngle);

        }
    }

    public void disableRotation() {
        rotation.disable();
        enabled = false;
    }

    public void enableRotation() {
        rotation.enable();
        enabled = true;
    }
}
