package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class OdometrySubsystem extends SubsystemBase {

    private Servo odometryLeft;
    private Servo odometryRight;
    private Servo odometryMiddle;

    private enum OdometryServos {
        Up,
        D
    }
}
