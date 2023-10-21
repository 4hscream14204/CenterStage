package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class airplaneLauncherSubsystem extends SubsystemBase {

    private Servo srvAirplaneLauncher;

    public airplaneLauncherSubsystem(Servo airplaneLauncherConstructor) {
        srvAirplaneLauncher = airplaneLauncherConstructor;
    }
        public void Raise () {
            srvAirplaneLauncher.setPosition(1);
        }
    }
