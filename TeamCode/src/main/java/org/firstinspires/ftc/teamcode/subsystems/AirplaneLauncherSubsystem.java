package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneLauncherSubsystem extends SubsystemBase {

    private Servo srvAirplaneLauncher;
    private Servo srvAirplaneLauncherEv;
    private enum AirplaneState {
        LOWER,
        RAISE,
        RELEASE
    }
    AirplaneState airplaneState;

   private double releaseSrvPos = 0.8;
   private double srvEVPosLower = 0.0;
   private double srvEVPosRaise = 0.2;
   private double srvLauncherPos = 1;


    public AirplaneLauncherSubsystem(Servo airplaneLauncherConstructor, Servo srvAirplaneLauncherEvCon) {
        srvAirplaneLauncher = airplaneLauncherConstructor;
        srvAirplaneLauncherEv = srvAirplaneLauncherEvCon;
        Lower();
    }
        public void Release () {
            srvAirplaneLauncher.setPosition(releaseSrvPos);
            airplaneState = AirplaneState.RELEASE;
        }

         public void Lower (){
        srvAirplaneLauncherEv.setPosition(srvEVPosLower);
        srvAirplaneLauncher.setPosition(srvLauncherPos);
        airplaneState = AirplaneState.LOWER;
    }
    public void Raise (){
        srvAirplaneLauncherEv.setPosition(srvEVPosRaise);
        airplaneState = AirplaneState.RAISE;
    }
    public void RaiseAndLaunch(){
        switch (airplaneState) {
            case LOWER:
                Raise();
                break;
            case RAISE:
                Release();
                break;
        }
    }
}

