package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangingMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;


public class RobotBase extends Object{

    // enum for alliance side
    public enum Alliance {
        RED,
        BLUE
    }

    public enum ChassisControlType {
        FIELDCENTRIC,
        ROBOTCENTRIC
    }
    public enum StartPosition {
        LEFT,
        RIGHT
    }
    public enum PropPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        NONE
    }
    public enum LeftClawState {
        OPEN,
        CLOSED
    }
    public enum RightClawState {
        OPEN,
        CLOSED
    }
    public enum LeftSlideHeight {
        GRABBING,
        LOW,
        MEDIUM,
        HIGH
    }
    public enum RightSlideHeight {
        GRABBING,
        LOW,
        MEDIUM,
        HIGH
    }
    public enum SyncSlidesMode {
        ON,
        OFF
    }
    public enum HangingState {
        RELEASED,
        DOWN
    }
    public enum AirplaneState {
        LOWER,
        RAISE,
        RELEASE
    }

    public DistanceSensor frontDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor backDistanceSensor;
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor rightFront;
    public DcMotor dcmIntake;
    public DcMotor dcmArm;
    public DcMotor dcmHangingMechanism;
    public Servo srvLeftClaw;
    public Servo srvRightClaw;
    public Servo srvAirplaneLauncher;
    public Servo srvAirplaneLauncherEv;
    public Servo srvOdometryLeft;
    public Servo srvOdometryRight;
    public Servo srvOdometryMiddle;
    public Servo srvLeftSlide;
    public Servo srvRightSlide;
    public TouchSensor redSwitch;
    public TouchSensor greenSwitch;
    public TouchSensor blackSwitch;
    public TouchSensor blueSwitch;
    public HuskyLens huskyLens;
    public IMU imu;
    public IntegratingGyroscope gyro;
    public NavxMicroNavigationSensor navxMicro;
    public ClawSubsystem clawSubsystem;
    public SlideSubsystem slideSubsystem;
    public AirplaneLauncherSubsystem airplaneLauncherSubsystem;
    public HangingMechanismSubsystem hangingMechanismSubsystem;
    public HuskyLensSubsystem huskyLensSubsystem;
    public SampleMecanumDrive mecanumDriveSubsystem;
    public IntakeSubsystem intakeSubsystem;

    // first instance of alliance
    public Alliance alliance;
    public ChassisControlType controlScheme;
    public StartPosition startPosition;
    public PropPosition propPosition;
    public LeftClawState leftClawState;
    public RightClawState rightClawState;
    public LeftSlideHeight leftSlideHeight;
    public RightSlideHeight rightSlideHeight;
    public SyncSlidesMode syncSlidesMode;
    public HangingState hangingState;
    public AirplaneState airplaneState;

    public RobotBase (HardwareMap hwMap) {
        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontDistance");
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistance");
        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistance");
        backDistanceSensor = hwMap.get(DistanceSensor.class, "backDistance");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        dcmIntake = hwMap.get(DcMotor.class, "Intake");
        srvLeftClaw = hwMap.get(Servo.class,"srvLeftClaw");
        srvRightClaw = hwMap.get(Servo.class,"srvRightClaw");
        dcmArm = hwMap.get(DcMotor.class,"dcmArm");
        dcmHangingMechanism = hwMap.get(DcMotor.class,"hangingMechanism");
        srvAirplaneLauncher = hwMap.get(Servo.class,"airplaneLauncher");
        srvAirplaneLauncherEv = hwMap.get(Servo.class, "airplaneLauncherEv");
        srvOdometryLeft = hwMap.get(Servo.class, "odometryLeft");
        srvOdometryRight = hwMap.get(Servo.class, "odometryRight");
        srvOdometryMiddle = hwMap.get(Servo.class, "odometryMiddle");
        navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx");
        huskyLens = hwMap.get(HuskyLens.class,"huskyLens");
        imu = hwMap.get(IMU.class,"imu");

        controlScheme = ChassisControlType.FIELDCENTRIC;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        gyro = (IntegratingGyroscope)navxMicro;

        clawSubsystem = new ClawSubsystem(srvLeftClaw, srvRightClaw);
        airplaneLauncherSubsystem = new AirplaneLauncherSubsystem(srvAirplaneLauncher, srvAirplaneLauncherEv);
        hangingMechanismSubsystem = new HangingMechanismSubsystem(dcmHangingMechanism);
        mecanumDriveSubsystem = new SampleMecanumDrive(hwMap);
        huskyLensSubsystem = new HuskyLensSubsystem(huskyLens);
        intakeSubsystem = new IntakeSubsystem(dcmIntake);

        //default value for the alliance side
        alliance = Alliance.RED;
        //default value for the chassis control type
        controlScheme = ChassisControlType.FIELDCENTRIC;
        //default value for the start position side
        startPosition = StartPosition.LEFT;
        //default value for the prop position
        propPosition = PropPosition.NONE;
        //default value for the sync slides mode
        syncSlidesMode = SyncSlidesMode.ON;
        //default value for hanging state
        hangingState = HangingState.DOWN;
        //default value for airplane state
        airplaneState = AirplaneState.LOWER;
    }
}
