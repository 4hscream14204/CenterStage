package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class TouchSensorSubsystem extends SubsystemBase {

    DigitalChannel tsIntake;

    public TouchSensorSubsystem (DigitalChannel intakeSensorConstructor) {
        tsIntake = intakeSensorConstructor;
        //tsIntake.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public boolean pixelInIntake(){
        boolean bolPixelInIntake = false;
        if(tsIntake.getState() == false){
            bolPixelInIntake = true;
        } else {
            bolPixelInIntake = false;
        }
        return bolPixelInIntake;
    }
}