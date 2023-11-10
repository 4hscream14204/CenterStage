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
        DownPosition();
        GrabOne();
    }
    public void Drop () {
        srvGrabber.setPosition(0);
        intPixelInGrabber = 0;
    }

    public void GrabOne () {
        srvGrabber.setPosition(1);
        intPixelInGrabber = 1;
    }

    public void GrabTwo () {
        srvGrabber.setPosition(0);
        intPixelInGrabber = 2;
    }

    public void ToggleGrabber () {
        if (intPixelInGrabber == 0) {
            srvGrabber.setPosition(1);
            intPixelInGrabber = 1;
        } else {
            srvGrabber.setPosition(0);
            intPixelInGrabber = 0;
        }
    }

    public void DropPosition () {
        srvArm.setPosition(0.33);
        bolDropPosToggle = true;
    }

    public void DownPosition () {
        srvArm.setPosition(0);
        bolDropPosToggle = false;
    }

    public void ToggleArm () {
        if (bolDropPosToggle) {
            srvArm.setPosition(0.38);
            bolDropPosToggle = false;
        } else {
            srvArm.setPosition(0);
            bolDropPosToggle = true;
        }
    }
}
