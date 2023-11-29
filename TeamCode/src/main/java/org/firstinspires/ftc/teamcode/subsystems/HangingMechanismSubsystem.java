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



    public HangingMechanismSubsystem(DcMotor hangingMechanismConstructor) {
        dcmHangingMechanism = hangingMechanismConstructor;
        lower();
    }

    public void initialRaisePosition() {
        dcmHangingMechanism.setTargetPosition(intHangingReleasePosition);
        hangingState = RobotBase.HangingState.RAISED;

    }

    public void lower() {
        dcmHangingMechanism.setTargetPosition(intHangingLowerPosition);
        hangingState = RobotBase.HangingState.LOWERED;
    }

    public void raise() {
        dcmHangingMechanism.setTargetPosition(intHangingRaisePosition);
        hangingState = RobotBase.HangingState.RAISED;
    }

    public void hangingToggle() {
        switch (hangingState) {
            case DOWN:
                initialRaisePosition();
                break;
            case RAISED:
                lower();
                break;
            case LOWERED:
                raise();
                break;
        }
    }

    public void hangingToggleCheck() {
        if (hangingState != RobotBase.HangingState.DOWN) {
            hangingToggle();
        }
    }
}