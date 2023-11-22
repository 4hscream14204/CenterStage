package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmSubsystem extends SubsystemBase {

    DcMotor dcmArm;
    private int intArmGrabbingPos = 0;
    private int intArmDropOffPos = 1;
    //537.7 PPR at the Output Shaft (312 RPM motor)

    public ArmSubsystem(DcMotor armConstructor) {
        dcmArm = armConstructor;
    }

    public void armGrabbingPosition() {
        dcmArm.setTargetPosition(intArmGrabbingPos);
    }

    public void armDropOffPos() {
        dcmArm.setTargetPosition(intArmDropOffPos);
    }
}
