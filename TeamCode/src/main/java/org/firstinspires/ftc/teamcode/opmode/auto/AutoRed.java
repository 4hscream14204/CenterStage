package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;



@Autonomous(name = "RedLeft")
public class AutoRed extends OpMode {
    public RobotBase robotBase;

    public AutoSuperClass autoSuperClass;

    @Override
    public void init(){



        robotBase = new RobotBase(hardwareMap);
       autoSuperClass.init();


    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){

    }
}
