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

    public OdometrySubsystem(Servo odometryLeftConstructor, Servo odometryRightConstructor, Servo odometryMiddleConstructor) {
        srvOdometryLeft = odometryLeftConstructor;
        srvOdometryMiddle = odometryMiddleConstructor;
        srvOdometryRight = odometryRightConstructor;
        //odometryStop();
        odometryState = OdometryState.DOWN;
    }

    public void odometryRaise() {
        srvOdometryLeft.setPosition(srvOdometryPosUp);
        srvOdometryMiddle.setPosition(srvOdometryPosUp);
        srvOdometryRight.setPosition(srvOdometryPosUp);
    }

    public void odometryLower() {
        srvOdometryLeft.setPosition(srvOdometryPosDown);
        srvOdometryMiddle.setPosition(srvOdometryPosDown);
        srvOdometryRight.setPosition(srvOdometryPosDown);
    }

    public void odometryToggle() {
        switch (odometryState) {
            case DOWN:
                odometryRaise();
                break;
            case UP:
                odometryLower();
                break;
        }
    }
    public void odometryStop() {
        srvOdometryLeft.setPosition(srvOdometryPosStop);
        srvOdometryMiddle.setPosition(srvOdometryPosStop);
        srvOdometryRight.setPosition(srvOdometryPosStop);
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

