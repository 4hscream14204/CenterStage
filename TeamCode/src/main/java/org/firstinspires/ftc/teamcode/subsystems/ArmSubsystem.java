package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class ArmSubsystem extends SubsystemBase {

    DcMotor dcmArm;
    public RobotBase.ArmState armState;

    public ArmSubsystem(DcMotor armConstructor) {
        dcmArm = armConstructor;
    }

    public void armGrabbingPosition() {
        armState = RobotBase.ArmState.GRABBING;
        dcmArm.setTargetPosition(armState.intArmPosition);
    }

    public void armDropOffPos() {
        armState = RobotBase.ArmState.DROPOFF;
        dcmArm.setTargetPosition(armState.intArmPosition);
    }
}
