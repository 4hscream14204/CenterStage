package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;



public class RobotBase extends Object{



    public DistanceSensor frontDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor backDistanceSensor;
    public DcMotor rightFrontMotor;
    public DcMotor leftFrontMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftBackMotor;
    public Servo srvDoubleCenterGrabber;
    public Servo srvArm;
    public Servo srvHangingMechanism;
    public Servo srvAirplaneLauncher;
    public HuskyLens huskyLens;
    //public IMU Gyro;

    public RobotBase (HardwareMap hwMap) {
        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontDistance");
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistance");
        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistance");
        backDistanceSensor = hwMap.get(DistanceSensor.class, "backDistance");
        rightFrontMotor = hwMap.get(DcMotor.class, "rightFrontMotor");
        leftFrontMotor = hwMap.get(DcMotor.class, "leftFrontMotor");
        rightBackMotor = hwMap.get(DcMotor.class, "rightBackMotor");
        leftBackMotor = hwMap.get(DcMotor.class, "leftBackMotor");
        srvDoubleCenterGrabber = hwMap.get(Servo.class,"srvGrabber");
        srvArm = hwMap.get(Servo.class,"srvArm");
        srvHangingMechanism = hwMap.get(Servo.class,"hangingMechanism");
        srvAirplaneLauncher = hwMap.get(Servo.class,"airplaneLauncher");
        huskyLens = hwMap.get(HuskyLens.class,"huskyLens");
        //Gyro = hwMap.get(IMU.class,"Gyro");

    }



}
