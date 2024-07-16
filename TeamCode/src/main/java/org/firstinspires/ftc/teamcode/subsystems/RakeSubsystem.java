package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class RakeSubsystem extends SubsystemBase {

    private Servo srvRake;

    private double dblRakePosition =

    public RakeSubsystem(Servo rakeConstructor){
        srvRake = rakeConstructor;
    }

    public void rakePosition(double dblRakePosition){
        srvRake.setPosition(dblRakePosition);
    }

}
