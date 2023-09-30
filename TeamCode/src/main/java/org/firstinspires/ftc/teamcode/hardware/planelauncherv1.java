package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "planelauncherv1")
public class planelauncherv1 extends LinearOpMode {

   private Servo srvPlaneLauncher;



    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

            srvPlaneLauncher = hardwareMap.servo.get("srvPlaneLauncher");

            srvPlaneLauncher.setPosition(1);

        }
    }
}