package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class HangingSensorSubsystem extends SubsystemBase {

    private DigitalChannel tsHangingSensor;

    public HangingSensorSubsystem(DigitalChannel tsHangingSensorConstructor) {
        tsHangingSensor = tsHangingSensorConstructor;
    }

    public boolean hangingIsDown(){
        boolean bolHangingIsDown = false;
        if(tsHangingSensor.getState() == false){
            bolHangingIsDown = true;
        } else {
            bolHangingIsDown = false;
        }
        return bolHangingIsDown;
    }
}
