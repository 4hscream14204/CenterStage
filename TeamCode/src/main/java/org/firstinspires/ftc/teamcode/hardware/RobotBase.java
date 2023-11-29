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
    public enum ClawState {
        OPEN,
        CLOSED
    }
    public enum SlideHeight {
        GRABBING (0),
        ESCAPE (0.1),
        LOWEST (0.2),
        LOW (0.3),
        LOWMEDIUM (0.4),
        MEDIUM (0.5),
        MEDIUMHIGH (0.6),
        HIGH (0.7),
        HIGHEST (0.8);
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
        RAISED (1),
        LOWERED (-1);
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
    //core hex motor At the output - 288 counts/revolution
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
    public ClawSubsystem leftClawSubsystem;
    public ClawSubsystem rightClawSubsystem;
    public SlideSubsystem rightSlideSubsystem;
    public SlideSubsystem leftSlideSubsystem;
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
    public SyncSlidesMode syncSlidesMode;

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

        leftClawSubsystem = new ClawSubsystem(srvLeftClaw, true);
        rightClawSubsystem = new ClawSubsystem(srvRightClaw, false);
        airplaneLauncherSubsystem = new AirplaneLauncherSubsystem(srvAirplaneLauncher, srvAirplaneLauncherEv);
        hangingMechanismSubsystem = new HangingMechanismSubsystem(dcmHangingMechanism);
        mecanumDriveSubsystem = new SampleMecanumDrive(hwMap);
        huskyLensSubsystem = new HuskyLensSubsystem(huskyLens);
        intakeSubsystem = new IntakeSubsystem(dcmIntake);
        leftSlideSubsystem = new SlideSubsystem(srvLeftSlide);
        rightSlideSubsystem = new SlideSubsystem(srvRightSlide);

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
