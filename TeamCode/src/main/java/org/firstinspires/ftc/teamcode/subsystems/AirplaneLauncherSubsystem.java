package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class AirplaneLauncherSubsystem extends SubsystemBase {

    private Servo srvAirplaneLauncher;
    private Servo srvAirplaneLauncherEv;
    private RobotBase.AirplaneState airplaneState;
    private double dblReleaseSrvPos = 0.8;
    private double dblEVPosLower = 0.0;
    private double dblEVPosRaise = 0.2;
    private double dblLoadedLauncherPos = 1;


    public AirplaneLauncherSubsystem(Servo airplaneLauncherConstructor, Servo LauncherElevatorConstructor) {
        srvAirplaneLauncher = airplaneLauncherConstructor;
        srvAirplaneLauncherEv = LauncherElevatorConstructor;
        lower();
    }
        public void release() {
            srvAirplaneLauncher.setPosition(dblReleaseSrvPos);
            airplaneState = RobotBase.AirplaneState.RELEASE;
        }

         public void lower(){
        srvAirplaneLauncherEv.setPosition(dblEVPosLower);
        srvAirplaneLauncher.setPosition(dblLoadedLauncherPos);
        airplaneState = RobotBase.AirplaneState.LOWER;
    }
    public void raise(){
        srvAirplaneLauncherEv.setPosition(dblEVPosRaise);
        airplaneState = RobotBase.AirplaneState.RAISE;
    }
    public void raiseAndLaunch(){
        switch (airplaneState) {
            case LOWER:
                raise();
                break;
            case RAISE:
                release();
                break;
        }
    }
}

