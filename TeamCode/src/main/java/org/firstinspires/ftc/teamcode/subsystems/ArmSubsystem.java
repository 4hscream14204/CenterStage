package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class ArmSubsystem extends SubsystemBase {

    DcMotor dcmArm;
    //goBilda 43 yellow jacket motor 3895.9 PPR at the Output Shaft
    public RobotBase.ArmState armState;
    private final double dblMainPower = 1;
    private final double dblSlowPower = 0.4;
    private final int intGrabbingPosition = 0;
    private final int intDropOffPosition = 1298;
    private final int intDropOffMediumHighPosition = 1200;
    private final int intDropOffLowestPosition = 1330;

    public ArmSubsystem(DcMotor armConstructor) {
        dcmArm = armConstructor;
        armState = RobotBase.ArmState.GRABBING;

        dcmArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmArm.setTargetPosition(0);
        dcmArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmArm.setPower(dblSlowPower);
    }

    public void armGrabbingPosition() {
        dcmArm.setPower(dblMainPower);
        armState = RobotBase.ArmState.GRABBING;
        dcmArm.setTargetPosition(intGrabbingPosition);
    }

    public void armDropOffPos() {
        dcmArm.setPower(dblMainPower);
        armState = RobotBase.ArmState.DROPOFF;
        dcmArm.setTargetPosition(intDropOffPosition);
        //telemetry.addLine("armDropOffPos Called");
    }

    public void armDropOffMHPos() {
        dcmArm.setPower(dblMainPower);
        armState = RobotBase.ArmState.DROPOFF;
        dcmArm.setTargetPosition(intDropOffMediumHighPosition);
    }

    public void armDropOffLowestPos() {
        dcmArm.setPower(dblMainPower);
        armState = RobotBase.ArmState.DROPOFF;
        dcmArm.setTargetPosition(intDropOffLowestPosition);
        //telemetry.addLine("armDropOffPos Called");
    }

    public int getArmPosition() {
        return dcmArm.getCurrentPosition();
    }
    public boolean armIsPassedWristSafe() {
        boolean bolArmIsPassedSafeDrop = false;
        if(getArmPosition() > 216) {
            bolArmIsPassedSafeDrop = true;
        }
        return bolArmIsPassedSafeDrop;
    }

    public boolean armIsPassedExtendSlideSafe() {
        boolean bolArmIsPassedSafeDrop = false;
        if(getArmPosition() > 600) {
            bolArmIsPassedSafeDrop = true;
        }
        return bolArmIsPassedSafeDrop;
    }

    public boolean armIsPassedSafeDrop() {
        boolean bolArmIsPassedSafeDrop = false;
        if(getArmPosition() > 1100) {
            bolArmIsPassedSafeDrop = true;
        }
        return bolArmIsPassedSafeDrop;
    }

    public boolean armIsInGrabbing() {
        boolean bolArmIsPassedSafeDrop = false;
        if(getArmPosition() < 10) {
            bolArmIsPassedSafeDrop = true;
        }
        return bolArmIsPassedSafeDrop;
    }

    public boolean armIsPassedReturnSlow() {
        boolean bolArmIsPassedReturnSlow = false;
        if(getArmPosition() < 500) {
            bolArmIsPassedReturnSlow = true;
        }
        return bolArmIsPassedReturnSlow;
    }

    public void armMainPower() {
        dcmArm.setPower(dblMainPower);
    }

    public void armSlowPower(){
        dcmArm.setPower(dblSlowPower);
    }

    public void armReturning() {
        armState = RobotBase.ArmState.RETURNING;
    }
}
