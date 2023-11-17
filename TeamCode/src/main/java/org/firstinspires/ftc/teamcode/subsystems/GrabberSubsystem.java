package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class GrabberSubsystem extends SubsystemBase{

    private Servo srvGrabber;
    private Servo srvArm;

    private boolean bolDropPosToggle = false;
    private int intPixelInGrabber = 1;

    public GrabberSubsystem(Servo grabberConstructor, Servo armConstructor) {
        srvGrabber = grabberConstructor;
        srvArm = armConstructor;
        downPosition();
        grabOne();
    }
    public void drop() {
        srvGrabber.setPosition(0);
        intPixelInGrabber = 0;
    }

    public void grabOne() {
        srvGrabber.setPosition(1);
        intPixelInGrabber = 1;
    }

    public void grabTwo() {
        srvGrabber.setPosition(0);
        intPixelInGrabber = 2;
    }

    public void toggleGrabber() {
        if (intPixelInGrabber == 0) {
            srvGrabber.setPosition(1);
            intPixelInGrabber = 1;
        } else {
            srvGrabber.setPosition(0);
            intPixelInGrabber = 0;
        }
    }

    public void dropPosition() {
        srvArm.setPosition(0.36);
        bolDropPosToggle = true;
    }

    public void downPosition() {
        srvArm.setPosition(0);
        bolDropPosToggle = false;
    }

    public void toggleArm() {
        if (bolDropPosToggle) {
            srvArm.setPosition(0.36);
            bolDropPosToggle = false;
        } else {
            srvArm.setPosition(0);
            bolDropPosToggle = true;
        }
    }
}
