package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.LED;

public class LightsSubsystem extends SubsystemBase {

    LED dgRedLight;
    LED dgGreenLight;

    public LightsSubsystem(LED redLightConstructor, LED greenLightConstructor){
        dgRedLight = redLightConstructor;
        dgGreenLight = greenLightConstructor;
        lightOff();
    }

    public void lightOn (){
        dgRedLight.enableLight(true);
        dgGreenLight.enableLight(true);
    }

    public void lightOff (){
        dgRedLight.enableLight(false);
        dgGreenLight.enableLight(false);
    }
}
