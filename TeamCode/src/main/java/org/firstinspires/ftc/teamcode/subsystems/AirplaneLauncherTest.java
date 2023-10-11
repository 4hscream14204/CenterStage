package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneLauncherTest extends LinearOpMode{

    Servo airplanelauncher;

    @Override
    public void runOpMode(){

        if(gamepad1.dpad_down)
            airplanelauncher.setPosition(1);
    }
}


