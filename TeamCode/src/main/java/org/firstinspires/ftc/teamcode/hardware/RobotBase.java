package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.commands.UniversalGrabbingPosCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangingMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LogitechCameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwitchBoardSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TouchSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.openftc.easyopencv.OpenCvCamera;


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
    public enum ClawState {
        OPEN,
        CLOSED
    }
    public enum SlideHeight {
        GRABBING (0),
        ESCAPE (0),
        LOWEST (0),
        LOW (0.546),
        LOWMEDIUM (0.68),
        MEDIUM (0.827),
        MEDIUMHIGH (1);
        //HIGH (1),
        //HIGHEST (0.8);
        public final double dblSlidePos;

        SlideHeight(double slidePosConstructor) {
            this.dblSlidePos = slidePosConstructor;
        }
    }
    public enum SyncSlidesMode {
        ON,
        OFF
    }
    public enum HangingState {
        DOWN (0),
        RAISED (2000),
        LOWERED (800);
        public final int intHangingPos;

        HangingState(int hangingPosConstructor) {
            this.intHangingPos = hangingPosConstructor;
        }
    }
    public enum AirplaneState {
        LOWER,
        RAISE,
        RELEASE
    }
    public enum ArmState {
        GRABBING (0),
        DROPOFF (1298),
        RETURNING(0);

        public final int intArmPosition;

        ArmState(int armPositionConstructor) {
            this.intArmPosition = armPositionConstructor;
        }
    }
    public enum WristState {
        GRABBING (0.5),
        ESCAPE (0),
        DROPOFF (1);

        public final double dblWristState;

        WristState(double wristStateConstructor) {
            this.dblWristState = wristStateConstructor;
        }
    }

    public enum ParkSide {
        INNER,
        OUTER,
    }

    /*
    public DistanceSensor frontDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor backDistanceSensor;
    */

    public DigitalChannel tsLeftIntake;
    public DigitalChannel tsRightIntake;

    public LED dgRedLeftLight;
    public LED dgRedRightLight;
    public LED dgGreenLeftLight;
    public LED dgGreenRightLight;
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor rightFront;
    public DcMotor dcmIntake;
    public DcMotor dcmArm;
    //goBilda 312 yellow jacket motor 537.7 PPR at the Output Shaft
    //goBilda 312 yellow jacket motor positive power is counterclockwise rotation
    public DcMotor dcmHangingMechanism;
    //core hex motor At the output - 288 counts/revolution
    public Servo srvLeftClaw;
    public Servo srvRightClaw;
    public Servo srvAirplaneLauncher;
    public Servo srvAirplaneLauncherEv;
    /*
    public Servo srvOdometryLeft;
    public Servo srvOdometryRight;
    public Servo srvOdometryMiddle;
    */
    public Servo srvLeftSlide;
    public Servo srvRightSlide;
    public Servo srvLeftWrist;
    public Servo srvRightWrist;
    /*
    public TouchSensor tsRedSwitch;
    public TouchSensor tsGreenSwitch;
    public TouchSensor tsBlackSwitch;
    public TouchSensor tsBlueSwitch;
    */
    public HuskyLens huskyLens;
    public OpenCvCamera logitechCamera;
    public IMU imu;
    public IntegratingGyroscope gyro;
    public NavxMicroNavigationSensor navxMicro;
    public ClawSubsystem leftClawSubsystem;
    public ClawSubsystem rightClawSubsystem;
    public SlideSubsystem rightSlideSubsystem;
    public SlideSubsystem leftSlideSubsystem;
    public AirplaneLauncherSubsystem airplaneLauncherSubsystem;
    public HangingMechanismSubsystem hangingMechanismSubsystem;
    public HuskyLensSubsystem huskyLensSubsystem;
    public SampleMecanumDrive mecanumDriveSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public WristSubsystem leftWristSubsystem;
    public WristSubsystem rightWristSubsystem;
    public SwitchBoardSubsystem redButtonSubsystem;
    public SwitchBoardSubsystem blueButtonSubsystem;
    public SwitchBoardSubsystem greenButtonSubsystem;
    public SwitchBoardSubsystem blackButtonSubsystem;
    public LightsSubsystem leftLightsSubsystem;
    public LightsSubsystem rightLightsSubsystem;
    public ArmSubsystem armSubsystem;
    public TouchSensorSubsystem leftTouchSensorSubsystem;
    public TouchSensorSubsystem rightTouchSensorSubsystem;

    public LogitechCameraSubsystem logitechCameraSubsystem;

    // first instance of alliance
    public Alliance alliance;
    public ChassisControlType controlScheme;
    public StartPosition startPosition;
    public PropPosition propPosition;
    public SyncSlidesMode syncSlidesMode;
    public ParkSide parkSide;

    public RobotBase (HardwareMap hwMap) {
        /*
        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontDistance");
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistance");
        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistance");
        backDistanceSensor = hwMap.get(DistanceSensor.class, "backDistance");
        */
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        srvLeftClaw = hwMap.get(Servo.class,"srvLeftClaw");
        srvRightClaw = hwMap.get(Servo.class,"srvRightClaw");
        srvLeftSlide = hwMap.get(Servo.class,"srvLeftSlide");
        srvRightSlide = hwMap.get(Servo.class,"srvRightSlide");
        srvLeftWrist = hwMap.get(Servo.class,"srvLeftWrist");
        srvRightWrist = hwMap.get(Servo.class,"srvRightWrist");
        dcmArm = hwMap.get(DcMotor.class,"dcmArm");
        dcmIntake = hwMap.get(DcMotor.class,"dcmIntake");
        dcmHangingMechanism = hwMap.get(DcMotor.class,"hangingMechanism");
        srvAirplaneLauncher = hwMap.get(Servo.class,"airplaneLauncher");
        srvAirplaneLauncherEv = hwMap.get(Servo.class, "airplaneLauncherEv");
        navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx");
        huskyLens = hwMap.get(HuskyLens.class,"huskyLens");
        imu = hwMap.get(IMU.class,"imu");
        dgRedLeftLight = hwMap.get(LED.class,"redLeftLight");
        dgRedRightLight = hwMap.get(LED.class,"redRightLight");
        dgGreenLeftLight = hwMap.get(LED.class,"greenLeftLight");
        dgGreenRightLight = hwMap.get(LED.class,"greenRightLight");
        tsLeftIntake = hwMap.get(DigitalChannel.class,"tsLeftIntake");
        tsRightIntake = hwMap.get(DigitalChannel.class,"tsRightIntake");

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        gyro = (IntegratingGyroscope)navxMicro;

        leftClawSubsystem = new ClawSubsystem(srvLeftClaw, true);
        rightClawSubsystem = new ClawSubsystem(srvRightClaw, false);
        airplaneLauncherSubsystem = new AirplaneLauncherSubsystem(srvAirplaneLauncher, srvAirplaneLauncherEv);
        hangingMechanismSubsystem = new HangingMechanismSubsystem(dcmHangingMechanism);
        mecanumDriveSubsystem = new SampleMecanumDrive(hwMap);
        huskyLensSubsystem = new HuskyLensSubsystem(huskyLens);
        intakeSubsystem = new IntakeSubsystem(dcmIntake);
        leftSlideSubsystem = new SlideSubsystem(srvLeftSlide);
        rightSlideSubsystem = new SlideSubsystem(srvRightSlide);
        leftWristSubsystem = new WristSubsystem(srvLeftWrist, true);
        rightWristSubsystem = new WristSubsystem(srvRightWrist, false);
        leftLightsSubsystem = new LightsSubsystem(dgRedLeftLight, dgGreenLeftLight);
        rightLightsSubsystem = new LightsSubsystem(dgRedRightLight, dgGreenRightLight);
        leftTouchSensorSubsystem = new TouchSensorSubsystem(tsLeftIntake);
        rightTouchSensorSubsystem = new TouchSensorSubsystem(tsRightIntake);
        logitechCameraSubsystem = new LogitechCameraSubsystem(startPosition);
        /*
        redButtonSubsystem = new SwitchBoardSubsystem(tsRedSwitch);
        blueButtonSubsystem = new SwitchBoardSubsystem(tsBlueSwitch);
        greenButtonSubsystem = new SwitchBoardSubsystem(tsGreenSwitch);
        blackButtonSubsystem = new SwitchBoardSubsystem(tsBlackSwitch);
        */
        armSubsystem = new ArmSubsystem(dcmArm);

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
    }
}
