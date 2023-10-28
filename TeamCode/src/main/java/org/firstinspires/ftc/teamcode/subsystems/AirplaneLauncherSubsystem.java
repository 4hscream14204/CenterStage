package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneLauncherSubsystem extends SubsystemBase {

    private Servo srvAirplaneLauncher;

    public AirplaneLauncherSubsystem(Servo airplaneLauncherConstructor) {
        srvAirplaneLauncher = airplaneLauncherConstructor;
    }
        public void Release () {
            srvAirplaneLauncher.setPosition(1);
        }
    }
