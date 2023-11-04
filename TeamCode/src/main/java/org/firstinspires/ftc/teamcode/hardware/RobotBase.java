package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabberSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangingMechanismSubsystem;


public class RobotBase extends Object{

    // enum for alliance side
    public enum Alliance {
        BLUE,
        RED,
    }

    public enum ChassisControlType {
        FIELDCENTRIC,
        ROBOTCENTRIC
    }


    public DistanceSensor frontDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor backDistanceSensor;
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor rightFront;
    public Servo srvDoubleCenterGrabber;
    public Servo srvArm;
    public Servo srvHangingMechanism1;
    public Servo srvHangingMechanism2;
    public Servo srvAirplaneLauncher;
    public Servo srvAirplaneLauncherEv;
    public HuskyLens huskyLens;
    public IMU imu;
    public GrabberSubsystem Grabber;
    public AirplaneLauncherSubsystem AirplaneLauncher;
    public HangingMechanismSubsystem HangingMechanism;
    public SampleMecanumDrive MecanumDrive;
    public ChassisControlType controlScheme;

    // first instance of alliance
    public Alliance alliance;

    public RobotBase (HardwareMap hwMap) {
        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontDistance");
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistance");
        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistance");
        backDistanceSensor = hwMap.get(DistanceSensor.class, "backDistance");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        srvDoubleCenterGrabber = hwMap.get(Servo.class,"srvGrabber");
        srvArm = hwMap.get(Servo.class,"srvArm");
        srvHangingMechanism1 = hwMap.get(Servo.class,"hangingMechanism1");
        srvHangingMechanism2 = hwMap.get(Servo.class,"hangingMechanism2");
        srvAirplaneLauncher = hwMap.get(Servo.class,"airplaneLauncher");
        srvAirplaneLauncherEv = hwMap.get(Servo.class, "airplaneLauncherEv");
        huskyLens = hwMap.get(HuskyLens.class,"huskyLens");
        imu = hwMap.get(IMU.class,"imu");
        controlScheme = ChassisControlType.FIELDCENTRIC;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        Grabber = new GrabberSubsystem(srvDoubleCenterGrabber, srvArm);
        AirplaneLauncher = new AirplaneLauncherSubsystem(srvAirplaneLauncher, srvAirplaneLauncherEv);
        HangingMechanism = new HangingMechanismSubsystem(srvHangingMechanism1, srvHangingMechanism2);
        MecanumDrive = new SampleMecanumDrive(hwMap);

        //default value for alliance side
        alliance = Alliance.RED;
    }
}
