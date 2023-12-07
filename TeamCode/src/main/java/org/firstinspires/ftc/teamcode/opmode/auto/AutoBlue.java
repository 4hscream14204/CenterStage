package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;



@Autonomous(name = "AutoBlue")
public class AutoBlue extends AutoSuperClass {

    @Override
    public void init(){

        super.init();
        this.robotBase.alliance = RobotBase.Alliance.BLUE;

    }
}
