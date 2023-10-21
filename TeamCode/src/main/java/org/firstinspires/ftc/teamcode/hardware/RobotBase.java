package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.airplaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.grabberSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hangingMechanismSubsystem;


public class RobotBase extends Object{



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
    public Servo srvHangingMechanism;
    public Servo srvAirplaneLauncher;
    public HuskyLens huskyLens;
    public IMU Gyro;
    public grabberSubsystem Grabber;
    public airplaneLauncherSubsystem AirplaneLauncher;
    public hangingMechanismSubsystem HangingMechanism;
    public SampleMecanumDrive MecanumDrive;

    public RobotBase (HardwareMap hwMap) {
        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontDistance");
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistance");
        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistance");
        backDistanceSensor = hwMap.get(DistanceSensor.class, "backDistance");
        rightFront = hwMap.get(DcMotor.class, "rightFrontMotor");
        leftFront = hwMap.get(DcMotor.class, "leftFrontMotor");
        rightRear = hwMap.get(DcMotor.class, "rightBackMotor");
        leftRear = hwMap.get(DcMotor.class, "leftBackMotor");
        srvDoubleCenterGrabber = hwMap.get(Servo.class,"srvGrabber");
        srvArm = hwMap.get(Servo.class,"srvArm");
        srvHangingMechanism = hwMap.get(Servo.class,"hangingMechanism");
        srvAirplaneLauncher = hwMap.get(Servo.class,"airplaneLauncher");
        huskyLens = hwMap.get(HuskyLens.class,"huskyLens");
        Gyro = hwMap.get(IMU.class,"Gyro");

        Grabber = new grabberSubsystem(srvDoubleCenterGrabber, srvArm);
        AirplaneLauncher = new airplaneLauncherSubsystem(srvAirplaneLauncher);
        HangingMechanism = new hangingMechanismSubsystem(srvHangingMechanism);
        MecanumDrive = new SampleMecanumDrive(hwMap);
    }



}
