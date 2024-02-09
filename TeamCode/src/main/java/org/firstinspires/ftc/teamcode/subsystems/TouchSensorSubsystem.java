package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class TouchSensorSubsystem extends SubsystemBase {

    DigitalChannel tsIntake;

    public TouchSensorSubsystem (DigitalChannel intakeSensorConstructor) {
        tsIntake = intakeSensorConstructor;
    }

    public boolean pixelInIntake(){
        boolean bolPixelInIntake = false;
        if(tsIntake.getState() == true){
            bolPixelInIntake = true;
        } else {
            bolPixelInIntake = false;
        }
        return bolPixelInIntake;
    }
}
