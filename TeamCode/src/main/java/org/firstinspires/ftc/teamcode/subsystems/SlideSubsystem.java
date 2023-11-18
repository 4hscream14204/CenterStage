package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class SlideSubsystem extends SubsystemBase {

    Servo slideRight;
    Servo slideLeft;
    private RobotBase.LeftSlideHeight leftSlideHeight;
    private RobotBase.RightSlideHeight rightSlideHeight;
    private double dblSlideDownPos = 0;
    private double dblSlideLowPos = 0.3;
    private double dblSlideMediumPos = 0.6;
    private double dblSlideHighPos = 1;

    public SlideSubsystem(Servo leftSlideConstructor, Servo rightSlideConstructor) {
        slideLeft = leftSlideConstructor;
        slideRight = rightSlideConstructor;
    }

    public void slideLeftDown() {
        slideLeft.setPosition(dblSlideDownPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.DOWN;
    }

    public void slideLeftLow() {
        slideLeft.setPosition(dblSlideLowPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
    }

    public void slideLeftMedium() {
        slideLeft.setPosition(dblSlideMediumPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
    }

    public void slideLeftHigh() {
        slideLeft.setPosition(dblSlideHighPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
    }

    public void slideRightDown() {
        slideRight.setPosition(dblSlideHighPos);
        rightSlideHeight = RobotBase.RightSlideHeight.DOWN;
    }

    public void slideRightLow() {
        slideRight.setPosition(dblSlideLowPos);
        rightSlideHeight = RobotBase.RightSlideHeight.LOW;
    }

    public void slideRightMedium() {
        slideRight.setPosition(dblSlideMediumPos);
        rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
    }

    public void slideRightHigh() {
        slideRight.setPosition(dblSlideHighPos);
        rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
    }

    public void leftLowToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.LOW) {
            slideLeftDown();
        } else {
            slideLeftLow();
        }
    }
    public void leftMediumToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.MEDIUM) {
            slideLeftDown();
        } else {
            slideLeftMedium();
        }
    }
    public void leftHighToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.HIGH) {
            slideLeftDown();
        } else {
            slideLeftHigh();
        }
    }
    public void rightLowToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.LOW) {
            slideRightDown();
        } else {
            slideRightLow();
        }
    }
    public void rightMediumToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.MEDIUM) {
            slideRightDown();
        } else {
            slideRightMedium();
        }
    }
    public void rightHighToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.HIGH) {
            slideRightDown();
        } else {
            slideRightHigh();
        }
    }
}