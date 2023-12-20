package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class HangingMechanismSubsystem extends SubsystemBase {

    DcMotor dcmHangingMechanism;
    public RobotBase.HangingState hangingState;



    public HangingMechanismSubsystem(DcMotor hangingMechanismConstructor) {
        dcmHangingMechanism = hangingMechanismConstructor;
        hangingState = RobotBase.HangingState.DOWN;
        //CHANGE TO INITIALIZE DOWN
    }

    public void initialRaisePosition() {
        hangingState = RobotBase.HangingState.RAISED;
        dcmHangingMechanism.setTargetPosition(hangingState.intHangingPos);
    }

    public void lower() {
        hangingState = RobotBase.HangingState.LOWERED;
        dcmHangingMechanism.setTargetPosition(hangingState.intHangingPos);
    }

    public void raise() {
        hangingState = RobotBase.HangingState.RAISED;
        dcmHangingMechanism.setTargetPosition(hangingState.intHangingPos);
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