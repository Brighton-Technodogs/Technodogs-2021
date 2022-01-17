// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight {
    private String tableName;
    private NetworkTable limelightTable;

    /**
     * All possible modes that the Limelight's LED can be in.
     */
    public enum LimeLedMode {

        /**
         * Use the LED mode specified in the current pipeline.
         */
        PIPELINE, 

        /**
         * Turn the LED off.
         */
        OFF, 

        /**
         * Makes the LED Blink.
         */
        BLINK, 

        /**
         * Turns the LED On.
         */
        ON
    }

    /**
     * All possible modes that the Limelight's camera can be in.
     */
    public enum LimeCamMode {

        /**
         * Enable Vison Processing. (Default)
         */
        VISION, 
        
        /**
         * Act like a normal camera. (Note: disables vision processing)
         */
        NORMAL
    }

    /**
     * All the possible picture in picture modes that the limelight can be in.
     * Note that this only works if a webcam is plugged in to the Limelight's USB Port.
     */
    public enum LimeStreamMode {

        /**
         * Show streams side-by-side
         */
        STANDARD, 
        
        /**
         * Secondary camera is placed in the lower-right corner of the primary camera view
         */
        PIP_MAIN, 
        /**
         * Primary camera is placed in the lower-right corner of the primary camera view
         */
        PIP_SECONDARY
    }

    /**
     * Creates a new Limelight with the default network table name of "limelight"
     */
    public Limelight() {
        this.tableName = "limelight";
        this.limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
    }

    /**
     * Creates a new Limelight with a custom table name
     * @param tableName The limelight's NetworkTables name
     */
    public Limelight(String tableName) {
        this.tableName = tableName;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
    }

    /**
     * Determines if the limelight has any valid targets to track
     * @return True if any valid targets are visible
     */
    public Boolean targetVisible() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Get the horizontal offset of the target in the camera's view from -29.8 to 29.8 degrees
     * @return The horizontal offset
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    /**
     * Get the vertical offset of the target in the camera's view from -24.85 to 24.85 degrees
     * @return The vertical offset
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    /**
     * Get the area of the target as a percentage of the image size (0-100%)
     * @return The area of the target
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }

    /**
     * The skew of the target (-90 degrees to 0 degrees)
     * @return The skew of the target
     */
    public double getTargetSkew() {
        return limelightTable.getEntry("ts").getDouble(0);
    }

    /**
     * The time taken for the vision pipeline to process the image from the camera in ms
     * @return The pipeline latency
     */
    public double getPipelineLatency() {
        return limelightTable.getEntry("tl").getDouble(0);
    }

    /**
     * The overall latency of the limelight's data. Inlcudes the time taken to capture and process the image.
     * @return The limelight's latency
     */
    public double getOverallLatency() {
        return getPipelineLatency()+11;
    }

    /**
     * The length of the shortest side of the target's bounding box in pixels
     * @return The shortest side length
     */
    public double getShortSideLength() {
        return limelightTable.getEntry("tshort").getDouble(0);
    }

    /**
     * The length of the longest side of the target's bounding box in pixels
     * @return The longest side length
     */
    public double getLongSideLength() {
        return limelightTable.getEntry("tlong").getDouble(0);
    }

    /**
     * The width of the target in pixels
     * @return Target width
     */
    public double getTargetWidth() {
        return limelightTable.getEntry("thor").getDouble(0);
    }

    /**
     * The hight of the target in pixels
     * @return Target height
     */
    public double getTargetHeight() {
        return limelightTable.getEntry("tvert").getDouble(0);
    }

    /**
     * The active pipeline number (0-9)
     * @return Pipeline number
     */
    public double getActivePipeline() {
        return limelightTable.getEntry("getpipe").getDouble(0);
    }

    public double translation() {
        return limelightTable.getEntry("camtran").getDouble(0);
    }

    public LimeLedMode getLED() {
        int mode = (int) limelightTable.getEntry("ledMode").getDouble(0);
        switch (mode) {
            case 0:
                return LimeLedMode.PIPELINE;
        
            case 1:
                return LimeLedMode.OFF;

            case 2:
                return LimeLedMode.BLINK;
            
            case 3:
                return LimeLedMode.ON;
            default:
                return LimeLedMode.PIPELINE;
        }
    }

    /**
     * Sets the mode of the limelight's LED Array
     * @param mode see {@link LimeLedMode}
     */
    public void setLED(LimeLedMode mode) {
        double finalMode = 0;
        switch (mode) {
            case PIPELINE:
                finalMode = 0;
                break;

            case OFF:
                finalMode = 1;
                break;
            
            case BLINK:
                finalMode = 2;
                break;

            case ON:
                finalMode = 3;
                break;
        
            default:
                System.out.println("Limelight Mode Invalid!");
                finalMode = 0;
                break;
        }
        limelightTable.getEntry("ledMode").setDouble(finalMode);
    }

    public LimeCamMode getCamMode() {
        int mode = (int) limelightTable.getEntry("camMode").getDouble(0);
        switch (mode) {
            case 0:
                return LimeCamMode.VISION;
            
            case 1:
                return LimeCamMode.NORMAL;
        
            default:
                return LimeCamMode.VISION;
        }
    }

    /**
     * Sets the mode of the limelight's camera
     * @param mode see {@link LimeCamMode}
     */
    public void setCamMode(LimeCamMode mode) {
        double finalMode = 0;
        switch (mode) {
            case VISION:
                finalMode = 0;
                break;

            case NORMAL:
                finalMode = 1;
                break;
        
            default:
                finalMode = 0;
                break;
        }
        limelightTable.getEntry("camMode").setDouble(finalMode);
    }

    /**
     * Sets the pipeline number of the limelight
     * @param pipeline The pipeline number
     */
    public void setPipeline(double pipeline) {
        limelightTable.getEntry("pipeline").setDouble(pipeline);
    }

    /**
     * Sets the picture-in-picture mode of the limelight
     * @param sm see {@link LimeStreamMode}
     */
    public void setPiPMode(LimeStreamMode sm) {
        double finalSM = 0;
        switch (sm) {
            case STANDARD:
                finalSM = 0;
                break;

            case PIP_MAIN:
                finalSM = 1;
                break;
            
            case PIP_SECONDARY:
                finalSM = 2;
                break;
        
            default:
                finalSM = 0;
                break;
        }
        limelightTable.getEntry("stream").setDouble(finalSM);
    }

    /**
     * Sets whether snapshots should be taken from the camera 2 times per second
     * @param enable Enable snapshots
     */
    public void enableSnapshots(Boolean enable) {
        limelightTable.getEntry("snapshot").setDouble(enable ? 1 : 0);
    }

}
