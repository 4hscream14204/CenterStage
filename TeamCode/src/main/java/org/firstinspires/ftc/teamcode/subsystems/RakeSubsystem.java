package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class RakeSubsystem extends SubsystemBase {

    private Servo srvRake;

    public RakeSubsystem(Servo rakeConstructor){
        srvRake = rakeConstructor;
        rakePosition(0);
    }

    public void rakePosition(double dblRakePosition){
        srvRake.setPosition(dblRakePosition);
    }

}
