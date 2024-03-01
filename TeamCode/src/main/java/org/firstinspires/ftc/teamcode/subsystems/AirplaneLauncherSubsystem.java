package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class AirplaneLauncherSubsystem extends SubsystemBase {

    private Servo srvAirplaneLauncher;
    private Servo srvAirplaneLauncherEv;
    private RobotBase.AirplaneState airplaneState;
    private double dblReleaseSrvPos = 0.01;
    private double dblEVPosLower = 0.01;
    private double dblEVPosRaise = 0.182222;
    private double dblLoadedLauncherPos = 0;


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

    public boolean elevatorIsRaised(){
        if(airplaneState == RobotBase.AirplaneState.RAISE) {
            return true;
        }
        return false;
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

