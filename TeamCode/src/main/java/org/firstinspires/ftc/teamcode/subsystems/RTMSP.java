package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RTMSP extends LinearOpMode {

    private DcMotor rsptm;
    @Override
    public void runOpMode() throws InterruptedException {

        if(gamepad1.a)
            rsptm.setDirection(DcMotorSimple.Direction.FORWARD);

        if(gamepad1.x)
            rsptm.setDirection(DcMotorSimple.Direction.REVERSE);

        rsptm.setPower(gamepad1.left_trigger);
    }
}
