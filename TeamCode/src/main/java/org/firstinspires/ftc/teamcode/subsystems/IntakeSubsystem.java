package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem extends SubsystemBase {

    DcMotor dcmIntake;

    public IntakeSubsystem(DcMotor intakeConstructor) {
        dcmIntake = intakeConstructor;
    }

}
