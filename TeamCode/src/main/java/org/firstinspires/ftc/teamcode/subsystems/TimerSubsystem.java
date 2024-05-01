package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class TimerSubsystem extends SubsystemBase {
    private int intTimer;
    public TimerSubsystem(int timerConstructor){
        intTimer = timerConstructor;
    }
    public boolean timerIsPassed(int intTimerLength){
        boolean bolTimerIsPassed = false;
        if (intTimer == intTimerLength || intTimer > intTimerLength){
            bolTimerIsPassed = true;
        }
        return bolTimerIsPassed;
        }
    }
