package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimerSubsystem extends SubsystemBase {
    public boolean timerIsPassed(int intTimerLength, ElapsedTime timer){
        boolean bolTimerIsPassed = false;
        if (timer.milliseconds() == intTimerLength || timer.milliseconds() > intTimerLength){
            bolTimerIsPassed = true;
        }
        return bolTimerIsPassed;
        }
    }
