package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class HangingMechanismSubsystem extends SubsystemBase {

    DcMotor dcmHangingMechanism;
    public RobotBase.HangingState hangingState;

    private final double dblHangingPower = 0.2;



    public HangingMechanismSubsystem(DcMotor hangingMechanismConstructor) {
        dcmHangingMechanism = hangingMechanismConstructor;
        hangingState = RobotBase.HangingState.DOWN;

        //dcmHangingMechanism.setDirection(DcMotorSimple.Direction.REVERSE);
        dcmHangingMechanism.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmHangingMechanism.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmHangingMechanism.setTargetPosition(0);
        dcmHangingMechanism.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmHangingMechanism.setPower(0);
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
                dcmHangingMechanism.setPower(dblHangingPower);
            case LOWERED:
                raise();
                break;
            case RAISED:
                lower();
                break;
        }
    }

    public void hangingToggleCheck() {
        if (hangingState != RobotBase.HangingState.DOWN) {
            hangingToggle();
        }
    }
}