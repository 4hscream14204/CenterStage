package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.LED;

public class LightsSubsystem extends SubsystemBase {

    LED dgRedLight;
    LED dgGreenLight;

    public LightsSubsystem(LED redLightConstructor, LED greenLightConstructor){
        dgRedLight = redLightConstructor;
        dgGreenLight = greenLightConstructor;
        lightsOff();
    }

    public void lightsOn(){
        redLightOn();
        greenLightOn();
    }

    public void redLightOn(){
        dgRedLight.enableLight(true);
    }

    public void greenLightOn(){
        dgGreenLight.enableLight(true);
    }

    public void lightsOff (){
        redLightOff();
        greenLightOff();
    }

    public void redLightOff (){
        dgRedLight.enableLight(false);
    }

    public void greenLightOff (){
        dgGreenLight.enableLight(false);
    }
}
