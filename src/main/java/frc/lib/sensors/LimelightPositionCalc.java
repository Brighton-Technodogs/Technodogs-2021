// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.sensors;

/** Add your docs here. */
public class LimelightPositionCalc {
    private Limelight limelight;
    private double lensHeight;
    private double lensElevation;
    private double targetHeight;

    /**
     * Creates a new Limelight Position Calculator. Distance units do not matter as long as they are consistent throughout the program.
     * @param limelight The limelight instance used to calculate distance
     * @param lensHeight The height of the limelight's lens from the ground
     * @param lensElevation The angle (in degrees) of the limelight relative to straight forwards
     * @param targetHeight The height of the target
     */
    public LimelightPositionCalc(Limelight limelight, double lensHeight, double lensElevation, double targetHeight) {
        this.limelight = limelight;
        this.lensHeight = lensHeight;
        this.lensElevation = lensElevation;
    }

    /**
     * Calculates the distance to a target, if it is visible.
     * @return The distance if the target is visible, otherwise 0.
     */
    public double calculate() {
        if (limelight.targetVisible()){
        double lensToTarget = limelight.getVerticalOffset();
        double distance = (targetHeight-lensHeight) / Math.tan(Math.toRadians(lensElevation)+Math.toRadians(lensToTarget));
        return distance;
        } else {
            return 0;
        }
    }
}
