package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class GrabberSubsystem extends SubsystemBase{

    private Servo srvGrabber;
    private Servo srvArm;

    private boolean bolDropPosToggle = false;
    private int intPixelInGrabber = 1;
    private double dblGrabOnePosition = 1;
    private double dblGrabTwoPosition = 0;
    private double dblGrabberDropPosition = 0.5;
    private double dblArmDropPosition = 0;
    private double dblArmDownPosition = 1;

    public GrabberSubsystem(Servo grabberConstructor, Servo armConstructor) {
        srvGrabber = grabberConstructor;
        srvArm = armConstructor;
        DownPosition();
        GrabOne();
    }
    public void Drop () {
        srvGrabber.setPosition(dblGrabberDropPosition);
        intPixelInGrabber = 0;
    }

    public void GrabOne () {
        srvGrabber.setPosition(dblGrabOnePosition);
        intPixelInGrabber = 1;
    }

    public void GrabTwo () {
        srvGrabber.setPosition(dblGrabTwoPosition);
        intPixelInGrabber = 2;
    }

    public void ToggleGrabber () {
        if (intPixelInGrabber == 0) {
            srvGrabber.setPosition(dblGrabTwoPosition);
            intPixelInGrabber = 2;
        } else {
            srvGrabber.setPosition(dblGrabberDropPosition);
            intPixelInGrabber = 0;
        }
    }

    public void DropPosition () {
        srvArm.setPosition(dblArmDropPosition);
        bolDropPosToggle = true;
    }

    public void DownPosition () {
        srvArm.setPosition(dblArmDownPosition);
        bolDropPosToggle = false;
    }

    public void ToggleArm () {
        if (bolDropPosToggle) {
            srvArm.setPosition(dblArmDownPosition);
            bolDropPosToggle = false;
        } else {
            srvArm.setPosition(dblArmDropPosition);
            bolDropPosToggle = true;
        }
    }
}
