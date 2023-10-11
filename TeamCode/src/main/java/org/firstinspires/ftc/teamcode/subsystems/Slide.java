package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Slide extends LinearOpMode {

    DcMotor SlideMotorLeft;
    DcMotor SlideMotorRight;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.y)
                SlideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            SlideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

                if(gamepad1.b)
                    SlideMotorLeft.setDirection((DcMotorSimple.Direction.FORWARD));
                SlideMotorRight.setDirection((DcMotorSimple.Direction.FORWARD));

                SlideMotorLeft.setPower(gamepad1.right_trigger);
                SlideMotorRight.setPower(gamepad1.right_trigger);
        }
    }
}
