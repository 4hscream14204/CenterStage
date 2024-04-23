package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class RakeSubsystem extends SubsystemBase {

    private Servo srvRake;
    private double dblClawDown = 0.0;
    private double dblClawUp = 1.0;
}
