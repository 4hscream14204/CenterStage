package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class hangingMechanismSubsystem extends SubsystemBase {

    Servo srvHangingMechanism;



    public hangingMechanismSubsystem (Servo hangingMechanismConstructor) {
       srvHangingMechanism = hangingMechanismConstructor;
    }

    public void Raise() {
        srvHangingMechanism.setPosition(1);
    }
}

