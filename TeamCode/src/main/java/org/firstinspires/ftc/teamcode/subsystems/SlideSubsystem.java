package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class SlideSubsystem extends SubsystemBase {

    Servo srvSlide;
    public RobotBase.SlideHeight slideHeight;

    public SlideSubsystem(Servo slideConstructor) {
        srvSlide = slideConstructor;
    }

    public void slideGoToPos(RobotBase.SlideHeight slideHeightParam) {
        slideHeight = slideHeightParam;
        srvSlide.setPosition(slideHeight.dblSlidePos);
    }

    public void slideGoToPos() {
        srvSlide.setPosition(slideHeight.dblSlidePos);
    }

    public void slideLowToggle() {
        if (slideHeight == RobotBase.SlideHeight.LOWEST || slideHeight == RobotBase.SlideHeight.LOW) {
            slideHeight = RobotBase.SlideHeight.GRABBING;
        } else {
            slideHeight = RobotBase.SlideHeight.LOW;
        }
        slideGoToPos();
    }

    public void slideMediumToggle() {
        if (slideHeight == RobotBase.SlideHeight.LOWMEDIUM || slideHeight == RobotBase.SlideHeight.MEDIUM || slideHeight == RobotBase.SlideHeight.MEDIUMHIGH) {
                slideHeight = RobotBase.SlideHeight.GRABBING;
        } else {
            slideHeight = RobotBase.SlideHeight.MEDIUM;
        }
        slideGoToPos();
    }

    public void slideHighToggle() {
        if (slideHeight == RobotBase.SlideHeight.HIGH || slideHeight == RobotBase.SlideHeight.HIGHEST) {
            slideHeight = RobotBase.SlideHeight.GRABBING;
        } else {
            slideHeight = RobotBase.SlideHeight.HIGH;
        }
        slideGoToPos();
    }

    public void slideTuningUp() {
        switch (slideHeight) {
            case GRABBING:
                slideHeight = RobotBase.SlideHeight.LOWEST;
                slideGoToPos();
                break;
            case LOWEST:
                slideHeight = RobotBase.SlideHeight.LOW;
                slideGoToPos();
                break;
            case LOW:
                slideHeight = RobotBase.SlideHeight.LOWMEDIUM;
                slideGoToPos();
                break;
            case LOWMEDIUM:
                slideHeight = RobotBase.SlideHeight.MEDIUM;
                slideGoToPos();
                break;
            case MEDIUM:
                slideHeight = RobotBase.SlideHeight.MEDIUMHIGH;
                slideGoToPos();
                break;
            case MEDIUMHIGH:
                slideHeight = RobotBase.SlideHeight.HIGH;
                slideGoToPos();
                break;
            case HIGH:
                slideHeight = RobotBase.SlideHeight.HIGHEST;
                slideGoToPos();
                break;
        }
    }

    public void slideTuningDown() {
        switch (slideHeight) {
            case LOWEST:
                slideHeight = RobotBase.SlideHeight.GRABBING;
                slideGoToPos();
                break;
            case LOW:
                slideHeight = RobotBase.SlideHeight.LOWEST;
                slideGoToPos();
                break;
            case LOWMEDIUM:
                slideHeight = RobotBase.SlideHeight.LOW;
                slideGoToPos();
                break;
            case MEDIUM:
                slideHeight = RobotBase.SlideHeight.LOWMEDIUM;
                slideGoToPos();
                break;
            case MEDIUMHIGH:
                slideHeight = RobotBase.SlideHeight.MEDIUM;
                slideGoToPos();
                break;
            case HIGH:
                slideHeight = RobotBase.SlideHeight.MEDIUMHIGH;
                slideGoToPos();
                break;
            case HIGHEST:
                slideHeight = RobotBase.SlideHeight.HIGH;
                slideGoToPos();
                break;
        }
    }

}