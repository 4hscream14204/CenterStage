package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class HangingMechanismSubsystem extends SubsystemBase {

    Servo srvHangingMechanism1;
    Servo srvHangingMechanism2;
    private final double dblHangingReleasePosition = 0.5;
    private final double dblHangingClosePosition1 = 0;
    private final double dblHangingClosePosition2 = 1;



    public HangingMechanismSubsystem(Servo hangingMechanism1Constructor, Servo hangingMechanism2Constructor) {
        srvHangingMechanism1 = hangingMechanism1Constructor;
        srvHangingMechanism2 = hangingMechanism2Constructor;
        lower();
    }

    public void raise() {
        srvHangingMechanism1.setPosition(dblHangingReleasePosition);
        srvHangingMechanism2.setPosition(dblHangingReleasePosition);
    }
    public void lower() {
        srvHangingMechanism1.setPosition(dblHangingClosePosition1);
        srvHangingMechanism2.setPosition(dblHangingClosePosition2);
    }
}