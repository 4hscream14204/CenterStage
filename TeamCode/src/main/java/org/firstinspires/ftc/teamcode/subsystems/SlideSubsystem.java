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
    private double dblSlideLowestPos = 0.1;
    private double dblSlideLowPos = 0.3;
    private double dblSlideLowMediumPos = 0.4;
    private double dblSlideMediumPos = 0.6;
    private double dblSlideMediumHighPos = 0.7;
    private double dblSlideHighPos = 0.8;
    private double dblSlideTopPos = 1;

    public SlideSubsystem(Servo leftSlideConstructor, Servo rightSlideConstructor) {
        slideLeft = leftSlideConstructor;
        slideRight = rightSlideConstructor;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION FOR GRABBING PIXELS
    public void slideLeftDown() {
        slideLeft.setPosition(dblSlideDownPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.DOWN;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE LOWEST SET LINE
    public void slideLeftLow() {
        slideLeft.setPosition(dblSlideLowPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE MEDIUM SET LINE
    public void slideLeftMedium() {
        slideLeft.setPosition(dblSlideMediumPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE HIGH SET LINE
    public void slideLeftHigh() {
        slideLeft.setPosition(dblSlideHighPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION FOR GRABBING PIXELS
    public void slideRightDown() {
        slideRight.setPosition(dblSlideHighPos);
        rightSlideHeight = RobotBase.RightSlideHeight.DOWN;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE LOWEST SET LINE
    public void slideRightLow() {
        slideRight.setPosition(dblSlideLowPos);
        rightSlideHeight = RobotBase.RightSlideHeight.LOW;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE MEDIUM SET LINE
    public void slideRightMedium() {
        slideRight.setPosition(dblSlideMediumPos);
        rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE HIGH SET LINE
    public void slideRightHigh() {
        slideRight.setPosition(dblSlideHighPos);
        rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
    }

    //TOGGLES THE LEFT SLIDE BETWEEN GRABBING POSITION AND THE LOW POSITION
    public void leftLowToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.LOW) {
            slideLeftDown();
        } else {
            slideLeftLow();
        }
    }

    //TOGGLES THE LEFT SLIDE BETWEEN GRABBING POSITION AND THE MEDIUM POSITION
    public void leftMediumToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.MEDIUM) {
            slideLeftDown();
        } else {
            slideLeftMedium();
        }
    }

    //TOGGLES THE LEFT SLIDE BETWEEN GRABBING POSITION AND THE HIGH POSITION
    public void leftHighToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.HIGH) {
            slideLeftDown();
        } else {
            slideLeftHigh();
        }
    }

    //TOGGLES THE RIGHT SLIDE BETWEEN GRABBING POSITION AND THE LOW POSITION
    public void rightLowToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.LOW) {
            slideRightDown();
        } else {
            slideRightLow();
        }
    }

    //TOGGLES THE RIGHT SLIDE BETWEEN GRABBING POSITION AND THE MEDIUM POSITION
    public void rightMediumToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.MEDIUM) {
            slideRightDown();
        } else {
            slideRightMedium();
        }
    }

    //TOGGLES THE RIGHT SLIDE BETWEEN GRABBING POSITION AND THE HIGH POSITION
    public void rightHighToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.HIGH) {
            slideRightDown();
        } else {
            slideRightHigh();
        }
    }
}