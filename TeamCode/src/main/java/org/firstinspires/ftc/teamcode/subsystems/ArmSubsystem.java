package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class ArmSubsystem extends SubsystemBase {

    DcMotor dcmArm;
    //goBilda 312 yellow jacket motor 537.7 PPR at the Output Shaft
    //goBilda 312 yellow jacket motor positive power is counterclockwise rotation
    public RobotBase.ArmState armState;
    private final double dblGrabbingPower = 0.1;
    private final double dblLiftingPower = 0.3;
    private final int intGrabbingPosition = 0;
    private final int intDropOffPosition = 179;

    public ArmSubsystem(DcMotor armConstructor) {
        dcmArm = armConstructor;

        dcmArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmArm.setDirection(DcMotor.Direction.REVERSE);
        dcmArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmArm.setTargetPosition(0);
        dcmArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmArm.setPower(dblLiftingPower);
    }

    public void armGrabbingPosition() {
        dcmArm.setPower(dblGrabbingPower);
        armState = RobotBase.ArmState.GRABBING;
        dcmArm.setTargetPosition(intDropOffPosition);
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

    public boolean armIsPassedSafeDrop() {
        boolean bolArmIsPassedSafeDrop = false;
        if(dcmArm.getCurrentPosition() > 150) {
            bolArmIsPassedSafeDrop = true;
        }
        return bolArmIsPassedSafeDrop;
    }
}
