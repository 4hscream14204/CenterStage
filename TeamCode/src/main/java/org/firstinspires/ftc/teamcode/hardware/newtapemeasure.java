package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "newtapemeasure")
public class newtapemeasure extends LinearOpMode {
    private DcMotor TapeMeasure = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        TapeMeasure = hardwareMap.get(DcMotor.class, "TapeMeasure");

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            TapeMeasure.setPower(gamepad1.left_trigger);

            if(gamepad1.x)
                TapeMeasure.setDirection(DcMotor.Direction.REVERSE);

            if(gamepad1.y)
                TapeMeasure.setDirection(DcMotor.Direction.FORWARD);

            telemetry.update();
        }
    }
}
