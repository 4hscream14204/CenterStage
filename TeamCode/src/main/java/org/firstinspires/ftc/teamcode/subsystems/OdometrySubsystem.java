package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class OdometrySubsystem extends SubsystemBase {

    private Servo srvOdometryLeft;
    private Servo srvOdometryRight;
    private Servo srvOdometryMiddle;

    private enum OdometryState {
        UP,
        DOWN,
        STOP
    }

    OdometryState odometryState;
    private double srvOdometryPosUp = 1;
    private double srvOdometryPosDown = 0;
    private double srvOdometryPosStop = 0.5;

    public OdometrySubsystem(Servo srvOdometryLeftCon, Servo srvOdometryRightCon, Servo srvOdometryMiddleCon) {
        srvOdometryLeft = srvOdometryLeftCon;
        srvOdometryMiddle = srvOdometryMiddleCon;
        srvOdometryRight = srvOdometryRightCon;
        Stop();
        odometryState = OdometryState.DOWN;
    }

    public void Up() {
        srvOdometryLeft.setPosition(srvOdometryPosUp);
        srvOdometryMiddle.setPosition(srvOdometryPosUp);
        srvOdometryRight.setPosition(srvOdometryPosUp);
    }

    public void Stop() {
        srvOdometryLeft.setPosition(srvOdometryPosStop);
        srvOdometryMiddle.setPosition(srvOdometryPosStop);
        srvOdometryRight.setPosition(srvOdometryPosStop);
    }

    public void Down() {
        srvOdometryLeft.setPosition(srvOdometryPosDown);
        srvOdometryMiddle.setPosition(srvOdometryPosDown);
        srvOdometryRight.setPosition(srvOdometryPosDown);
    }

    public void OdometryToggle() {
        switch (odometryState) {
            case DOWN:
                Up();
                break;
            case UP:
                Down();
                break;
        }
    }
    public void OdometryStop() {
                Stop();
                switch (odometryState) {
                    case UP:
                        odometryState = OdometryState.DOWN;
                        break;
                    case DOWN:
                        odometryState = OdometryState.UP;
                        break;
                }
    }
}

