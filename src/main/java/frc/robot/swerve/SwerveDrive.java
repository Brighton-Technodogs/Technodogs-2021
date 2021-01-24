package frc.robot.swerve;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

    private SwerveWheel rightFrontWheel;
    private SwerveWheel leftFrontWheel;
    private SwerveWheel leftBackWheel;
    private SwerveWheel rightBackWheel;

    private GyroBase gyro;

    private double wheelbase = 22.5;
    private double trackwidth = 24.5;

    private double directionStickDeadZone = 0.08;
    private double rotateStickDeadZone = 0.08;

    private double slowModeSpeedMultiplier = 0.45;

    public SwerveDrive(SwerveWheel rightFront, SwerveWheel leftFront, SwerveWheel leftBack, SwerveWheel rightBack,
            GyroBase gyro) {
        this.rightFrontWheel = rightFront;
        this.leftFrontWheel = leftFront;
        this.leftBackWheel = leftBack;
        this.rightBackWheel = rightBack;

        System.out.println("SwerveDrive Initialized");

        this.gyro = gyro;
    }

    /**
     * Drives the robot based on parameter values
     * 
     * @param directionX Proportional speed at which to move left to right
     * @param directionY Proportional speed at which to move front to back
     * @param rotation   Proportional speed at which to rotate
     * @param useGyro    Boolean for field-oriented driving
     * @param slowSpeed  Boolean for slow mode to make the robot drive slower.
     * @param noPush     Boolean to lock wheels at 45 degree angles, to prevent the
     *                   robot from being pushed in any direction
     */
    public void drive(double directionX, double directionY, double rotation, boolean useGyro, boolean slowSpeed,
            boolean noPush) {

        // SmartDashboard.putNumber("directionX", directionX);
        // SmartDashboard.putNumber("directionY", directionY);
        // SmartDashboard.putNumber("rotation", rotation);
        // System.out.println(rotation);
        // System.out.println(directionX);
        // System.out.println(directionY);

        if (noPush) {
            // Puts Motors into X formation and disables motors
            this.rightFrontWheel.updateSpeed(0);
            this.leftFrontWheel.updateSpeed(0);
            this.leftBackWheel.updateSpeed(0);
            this.rightBackWheel.updateSpeed(0);

            this.rightFrontWheel.updateRotation(45 + 90);
            this.leftFrontWheel.updateRotation(315 + 90);
            this.leftBackWheel.updateRotation(225 + 90);
            this.rightBackWheel.updateRotation(135 + 90);
            return;
        }

        // Deadzone if BOTH joysticks are in the center
        if ((directionX < directionStickDeadZone && directionX > directionStickDeadZone*-1) && (directionY < directionStickDeadZone && directionY > directionStickDeadZone*-1)
                && (rotation < rotateStickDeadZone && rotation > rotateStickDeadZone*-1)) {
            this.rightFrontWheel.updateSpeed(0);
            this.leftFrontWheel.updateSpeed(0);
            this.leftBackWheel.updateSpeed(0);
            this.rightBackWheel.updateSpeed(0);

            this.rightFrontWheel.updateRotation();
            this.leftFrontWheel.updateRotation();
            this.leftBackWheel.updateRotation();
            this.rightBackWheel.updateRotation();
            return;
        }
        // Deadzone if ROTATION joystick only near the center (this fixes the rotation
        // drift)
        else if (rotation < rotateStickDeadZone && rotation > rotateStickDeadZone*-1) {
            rotation = 0;
        }

        double L = this.wheelbase; // distance between front and back wheels
        double W = this.trackwidth; // distance between front wheels
        double diameter = Math.sqrt((L * L) + (W * W)); // radius of circle (actually it may be the diameter?)

        directionY *= -1; // invert Y
        directionX *= -1; // invert X
        rotation *= -1;

        double a = directionX - rotation * (L / diameter); // rear axle
        double b = directionX + rotation * (L / diameter); // front axle
        double c = directionY - rotation * (W / diameter); // left track
        double d = directionY + rotation * (W / diameter); // right track

        /* !!!!!!!!!!! */
        /* SWERVE MATH */
        /* !!!!!!!!!!! */
        /*
         *                FRONT
         * 
         *            c          d
         *            | 		 |
         *       b ------------------ b
         *            |          |
         *            |          |
         * LEFT       |          |      RIGHT
         *            |          |
         *            |          |
         *       a ------------------ a
         *            |          |
         *            c          d
         * 
         *                BACK
         */

        // set motor speeds for each wheel
        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        // set wheel angle for each wheel
        double backRightAngle = (Math.atan2(a, d) / Math.PI) * 180;
        double backLeftAngle = (Math.atan2(a, c) / Math.PI) * 180;
        double frontRightAngle = (Math.atan2(b, d) / Math.PI) * 180;
        double frontLeftAngle = (Math.atan2(b, c) / Math.PI) * 180;

        // Field oriented
        if (useGyro) {
            double gyroAngle = normalizeGyroAngle(gyro.getAngle());
            backRightAngle += gyroAngle;
            backLeftAngle += gyroAngle;
            frontRightAngle += gyroAngle;
            frontLeftAngle += gyroAngle;
        }

        // Slow mode
        if (slowSpeed) {
            backRightSpeed *= slowModeSpeedMultiplier;
            backLeftSpeed *= slowModeSpeedMultiplier;
            frontRightSpeed *= slowModeSpeedMultiplier;
            frontLeftSpeed *= slowModeSpeedMultiplier;
        }

        // Prevents robot from being pushed.
        if (noPush) {
            // Puts Motors into X formation and disables motors
            frontRightAngle = 45 + 90;
            frontLeftAngle = 315 + 90;
            backLeftAngle = 225 + 90;
            backRightAngle = 135 + 90;

            backRightSpeed = 0;
            backLeftSpeed = 0;
            frontRightSpeed = 0;
            frontLeftSpeed = 0;

        }

        // update the commands to the motors
        this.rightFrontWheel.drive(frontRightSpeed, frontRightAngle);
        SmartDashboard.putNumber("front right commanded angle", frontRightAngle);
        this.leftFrontWheel.drive(frontLeftSpeed, frontLeftAngle);
        SmartDashboard.putNumber("front left commanded angle", frontRightAngle);
        this.leftBackWheel.drive(backLeftSpeed, backLeftAngle);
        SmartDashboard.putNumber("back left commanded angle", frontRightAngle);
        this.rightBackWheel.drive(backRightSpeed, backRightAngle);
        SmartDashboard.putNumber("back right commanded angle", frontRightAngle);

    }

    public double normalizeGyroAngle(double angle) {
        return (angle - (Math.floor(angle / 360) * 360));
    }

    public void driveSimple(double speed, double angle) {
        this.rightFrontWheel.drive(speed, angle);
        this.leftFrontWheel.drive(speed, angle);
        this.leftBackWheel.drive(speed, angle);
        this.rightBackWheel.drive(speed, angle);
    }

    //set the wheels so they are pointing forward clockwise. When passed a speed they spin in unison
    public void CircleRotate (double speed)
    {
        this.rightFrontWheel.drive(speed, 45);
        this.leftFrontWheel.drive(speed, 135);
        this.leftBackWheel.drive(speed, 225);
        this.rightBackWheel.drive(speed, 315);
    }

    public void XModeActivate()
    {
        this.rightFrontWheel.updateSpeed(0);
        this.leftFrontWheel.updateSpeed(0);
        this.leftBackWheel.updateSpeed(0);
        this.rightBackWheel.updateSpeed(0);

        this.rightFrontWheel.updateRotation(45 + 90);
        this.leftFrontWheel.updateRotation(315 + 90);
        this.leftBackWheel.updateRotation(225 + 90);
        this.rightBackWheel.updateRotation(135 + 90);
    }
}
