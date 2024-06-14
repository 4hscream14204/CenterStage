package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.LED;

public class LightsSubsystem extends SubsystemBase {

    LED dgRedLight;
    LED dgGreenLight;

    /*
    public LightsSubsystem(LED redLightConstructor, LED greenLightConstructor){
        dgRedLight = redLightConstructor;
        dgGreenLight = greenLightConstructor;
    }
    */
    public void lightsOn(LED redLightConstructor, LED greenLightConstructor){
        redLightOn(redLightConstructor);
        greenLightOn(greenLightConstructor);
    }

    public void redLightOn(LED redLightConstructor){
        dgRedLight = redLightConstructor;
        dgRedLight.enable(true);
    }

    public void greenLightOn(LED greenLightConstructor){
        dgGreenLight = greenLightConstructor;
        dgGreenLight.enable(true);
    }

    public void lightsOff (LED redLightConstructor, LED greenLightConstructor){
        redLightOff(redLightConstructor);
        greenLightOff(greenLightConstructor);
    }

    public void redLightOff (LED redLightConstructor){
        dgRedLight = redLightConstructor;
        dgRedLight.enable(false);
    }

    public void greenLightOff (LED greenLightConstructor){
        dgGreenLight = greenLightConstructor;
        dgGreenLight.enable(false);
    }
}
