package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class ArmSubsystem extends SubsystemBase {

    DcMotor dcmArm;
    //goBilda 43 yellow jacket motor 3895.9 PPR at the Output Shaft
    public RobotBase.ArmState armState;
    private final double dblGrabbingPower = 0.5;
    private final double dblLiftingPower = 0.6;
    private final int intGrabbingPosition = 0;
    private final int intDropOffPosition = 1298;

    public ArmSubsystem(DcMotor armConstructor) {
        dcmArm = armConstructor;
        armState = RobotBase.ArmState.GRABBING;

        dcmArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmArm.setTargetPosition(0);
        dcmArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmArm.setPower(dblLiftingPower);
    }

    public void armGrabbingPosition() {
        dcmArm.setPower(dblGrabbingPower);
        armState = RobotBase.ArmState.GRABBING;
        dcmArm.setTargetPosition(intGrabbingPosition);
    }

    public void armDropOffPos() {
        dcmArm.setPower(dblLiftingPower);
        armState = RobotBase.ArmState.DROPOFF;
        dcmArm.setTargetPosition(intDropOffPosition);
        //telemetry.addLine("armDropOffPos Called");
    }

    public int getArmPosition() {
        return dcmArm.getCurrentPosition();
    }
    public boolean armIsPassedWristSafe() {
        boolean bolArmIsPassedSafeDrop = false;
        if(dcmArm.getCurrentPosition() > 216) {
            bolArmIsPassedSafeDrop = true;
        }
        return bolArmIsPassedSafeDrop;
    }

    public boolean armIsPassedExtendSlideSafe() {
        boolean bolArmIsPassedSafeDrop = false;
        if(dcmArm.getCurrentPosition() > 1000) {
            bolArmIsPassedSafeDrop = true;
        }
        return bolArmIsPassedSafeDrop;
    }

    public boolean armIsPassedSafeDrop() {
        boolean bolArmIsPassedSafeDrop = false;
        if(dcmArm.getCurrentPosition() > 1100) {
            bolArmIsPassedSafeDrop = true;
        }
        return bolArmIsPassedSafeDrop;
    }

    public void armReturning() {
        armState = RobotBase.ArmState.RETURNING;
    }
}
