package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class HangingMechanismSubsystem extends SubsystemBase {

    DcMotor dcmHangingMechanism;
    private RobotBase.HangingState hangingState;
    private final int intHangingReleasePosition = 1;
    private final int intHangingLowerPosition = -1;
    private final int intHangingRaisePosition= 2;
    //At the output - 288 counts/revolution (core hex motor)



    public HangingMechanismSubsystem(DcMotor hangingMechanismConstructor) {
        dcmHangingMechanism = hangingMechanismConstructor;
        lower();
    }

    public void raisePosition() {
        dcmHangingMechanism.setTargetPosition(intHangingReleasePosition);
        hangingState = RobotBase.HangingState.RELEASED;

    }

    public void lower() {
        dcmHangingMechanism.setTargetPosition(intHangingLowerPosition);
    }

    public void raise() {
        dcmHangingMechanism.setTargetPosition(intHangingRaisePosition);
    }
}