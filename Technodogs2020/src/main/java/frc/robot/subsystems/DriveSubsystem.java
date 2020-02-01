package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc3707.Creedence.Robot;
import org.usfirst.frc3707.Creedence.Configuration.Constants;
import org.usfirst.frc3707.Creedence.commands.drive.DriveCommand;
import org.usfirst.frc3707.Creedence.pixy2API.Pixy2Line;
import org.usfirst.frc3707.Creedence.pixy2API.Pixy2Line.Vector;
import org.usfirst.frc3707.Creedence.swerve.SwerveDrive;
import org.usfirst.frc3707.Creedence.swerve.SwerveWheel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends Subsystem {

    private AnalogPotentiometer frontRightEncoder = new AnalogPotentiometer(Constants.DriveSystem.FrontRight.getEncoder(), 360.0, 0.0);
    private VictorSP frontRightSwerve = new VictorSP(Constants.DriveSystem.FrontRight.getSwerve());
    private PIDController frontRightTwist = new PIDController(0.05, 0.0, 0.0, 0.0, frontRightEncoder, frontRightSwerve, 0.02);
    private AnalogPotentiometer frontLeftEncoder = new AnalogPotentiometer(Constants.DriveSystem.FrontLeft.getEncoder(), 360.0, 0.0);;
    private VictorSP frontLeftSwerve = new VictorSP(Constants.DriveSystem.FrontLeft.getSwerve());
    private PIDController frontLeftTwist = new PIDController(0.05, 0.0, 0.0, 0.0, frontLeftEncoder, frontLeftSwerve, 0.02);;
    private AnalogPotentiometer backRightEncoder = new AnalogPotentiometer(Constants.DriveSystem.BackRight.getEncoder(), 360.0, 0.0);
    private VictorSP backRightSwerve = new VictorSP(Constants.DriveSystem.BackRight.getSwerve());
    private PIDController backRightTwist = new PIDController(0.05, 0.0, 0.0, 0.0, backRightEncoder, backRightSwerve, 0.02);
    private AnalogPotentiometer backLeftEncoder = new AnalogPotentiometer(Constants.DriveSystem.BackLeft.getEncoder(), 360.0, 0.0);
    private VictorSP backLeftSwerve = new VictorSP(Constants.DriveSystem.BackLeft.getSwerve());
    private PIDController backLeftTwist = new PIDController(0.05, 0.0, 0.0, 0.0, backLeftEncoder, backLeftSwerve, 0.02);
    
    private VictorSP frontRightDrive = new VictorSP(Constants.DriveSystem.FrontRight.getDrive());
    private VictorSP frontLeftDrive = new VictorSP(Constants.DriveSystem.FrontLeft.getDrive());
    private VictorSP backRightDrive = new VictorSP(Constants.DriveSystem.BackRight.getDrive());
    private VictorSP backLeftDrive = new VictorSP(Constants.DriveSystem.BackLeft.getDrive());

    // private CANSparkMax frontRightDrive = new CANSparkMax(Constants.DriveSystem.FrontRight.getDrive(), MotorType.kBrushless);
    // private CANSparkMax frontLeftDrive = new CANSparkMax(Constants.DriveSystem.FrontLeft.getDrive(), MotorType.kBrushless);
    // private CANSparkMax backRightDrive = new CANSparkMax(Constants.DriveSystem.BackRight.getDrive(), MotorType.kBrushless);
    // private CANSparkMax backLeftDrive = new CANSparkMax(Constants.DriveSystem.BackLeft.getDrive(), MotorType.kBrushless);


    private SwerveWheel frontRightWheel = new SwerveWheel(frontRightTwist, frontRightDrive, Constants.DriveSystem.FrontRight.getOffset());
    private SwerveWheel frontLeftWheel = new SwerveWheel(frontLeftTwist, frontLeftDrive,Constants.DriveSystem.FrontLeft.getOffset());
    private SwerveWheel backRightWheel = new SwerveWheel(backRightTwist, backRightDrive, Constants.DriveSystem.BackRight.getOffset());
    private SwerveWheel backLeftWheel = new SwerveWheel(backLeftTwist, backLeftDrive, Constants.DriveSystem.BackLeft.getOffset());
    public SwerveDrive swerve = new SwerveDrive(frontRightWheel, frontLeftWheel, backLeftWheel, backRightWheel, null);

    public void init() {
        //this is how you set a parameter on the spark... this one sets it to PWM
        //frontLeftDrive.setParameter(com.revrobotics.CANSparkMaxLowLevel.ConfigParameter.kInputMode, CANSparkMax.InputMode.kPWM.value);
        
        frontRightTwist.setInputRange(0.0, 360.0);
        frontRightTwist.setOutputRange(-1.0, 1.0);
        frontRightTwist.setContinuous(true);

        frontLeftTwist.setInputRange(0.0, 360.0);
        frontLeftTwist.setOutputRange(-1.0, 1.0);
        frontLeftTwist.setContinuous(true);

        backLeftTwist.setInputRange(0.0, 360.0);
        backLeftTwist.setOutputRange(-1.0, 1.0);
        backLeftTwist.setContinuous(true);

        backRightTwist.setInputRange(0.0, 360.0);
        backRightTwist.setOutputRange(-1.0, 1.0);
        backRightTwist.setContinuous(true);

    }

    /**
     * Enables motors. bcuz it werkz?
     */
    public void enable() {
        frontRightWheel.enableRotation();
        frontLeftWheel.enableRotation();
        backRightWheel.enableRotation();
        backLeftWheel.enableRotation();
    }

    /**
     * Disables motors. bcuz it werkz?
     */
    public void disable() {
        frontRightWheel.disableRotation();
        frontLeftWheel.disableRotation();
        backRightWheel.disableRotation();
        backLeftWheel.disableRotation();
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
        swerve.drive(directionX, directionY, rotation, false, slowSpeed, noPush);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }

    /**
     * The function which executes periodically to run the DriveTrain subsystem
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Right Encoder", frontRightEncoder.get());
        SmartDashboard.putNumber("Front Left Encoder", frontLeftEncoder.get());
        SmartDashboard.putNumber("Back Right Encoder", backRightEncoder.get());
        SmartDashboard.putNumber("Back Left Encoder", backLeftEncoder.get());
    }

    public void disableFrontRightWheelRotation(){
        frontRightWheel.disableRotation();
    }
    public void disableFrontLeftWheelRotation(){
        frontLeftWheel.disableRotation();
    }
    public void disableBackRightWheelRotation(){
        backRightWheel.disableRotation();
    }
    public void disableBackLeftWheelRotation(){
        backLeftWheel.disableRotation();
    }
    public void enableFrontRightWheelRotation(){
        frontRightWheel.enableRotation();
    }
    public void enableFrontLeftWheelRotation(){
        frontLeftWheel.enableRotation();
    }
    public void enableBackRightWheelRotation(){
        backRightWheel.enableRotation();
    }
    public void enableBackLeftWheelRotation(){
        backLeftWheel.enableRotation();
    }
}
