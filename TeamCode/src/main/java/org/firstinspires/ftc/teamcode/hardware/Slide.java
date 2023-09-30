package org.firstinspires.ftc.teamcode.RI3D;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Slide")
public class Slide extends LinearOpMode {
    private DcMotor SlideMotorLeft = null;

    private DcMotor SlideMotorRight = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SlideMotorLeft = hardwareMap.get(DcMotor.class, "SlideMotorLeft");
        SlideMotorRight = hardwareMap.get(DcMotor.class,"SlideMotorRight");

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            SlideMotorLeft.setPower(gamepad1.right_trigger);
            SlideMotorRight.setPower(gamepad1.right_trigger);

            if(gamepad1.a)
                SlideMotorLeft.setDirection(DcMotor.Direction.FORWARD);

            if(gamepad1.a)
                      SlideMotorRight.setDirection(DcMotor.Direction.REVERSE);

            if(gamepad1.b)
                SlideMotorLeft.setDirection(DcMotor.Direction.REVERSE);

            if(gamepad1.b)
                SlideMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
            telemetry.update();
        }
    }
}
