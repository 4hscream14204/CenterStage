package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class RakeSubsystem extends SubsystemBase {

    private Servo srvRake;
    private double dblRakeDown = 0.0;
    private double dblRakeUp = 1.0;

    public RakeSubsystem(Servo rakeConstructor){
        srvRake = rakeConstructor;
    }

    public void rakeUp(){
        srvRake.setPosition(dblRakeUp);
    }

    public void rakeDown(){
        srvRake.setPosition(dblRakeDown);
    }
}
