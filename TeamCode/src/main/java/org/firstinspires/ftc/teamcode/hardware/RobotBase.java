package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TimerSubsystem;

public class RobotBase extends Object {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor dcmIntake;
    public TimerSubsystem timerSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public LightsSubsystem lightsSubsystem;
    public RobotBase (HardwareMap hwMap) {
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");

        intakeSubsystem = new IntakeSubsystem(dcmIntake);

    }

}
