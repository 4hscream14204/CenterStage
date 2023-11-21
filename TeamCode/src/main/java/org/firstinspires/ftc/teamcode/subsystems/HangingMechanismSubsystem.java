package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class HangingMechanismSubsystem extends SubsystemBase {

    DcMotor dcmHangingMechanism;
    private final int intHangingReleasePosition = 1;
    private final int intHangingLowerPosition = -1;
    private final int intHangingRaisePosition= 2;



    public HangingMechanismSubsystem(DcMotor hangingMechanismConstructor) {
        dcmHangingMechanism = hangingMechanismConstructor;
        lower();
    }

    public void raisePosition() {
        dcmHangingMechanism.setTargetPosition(intHangingReleasePosition);
    }
    public void lower() {
        dcmHangingMechanism.setTargetPosition(intHangingLowerPosition);
    }

    public void raise() {
        dcmHangingMechanism.setTargetPosition(intHangingRaisePosition);
    }
}