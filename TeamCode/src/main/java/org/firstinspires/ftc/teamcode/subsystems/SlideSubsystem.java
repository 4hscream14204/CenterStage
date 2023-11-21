package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class SlideSubsystem extends SubsystemBase {

    Servo srvSlideRight;
    Servo srvSlideLeft;
    private RobotBase.LeftSlideHeight leftSlideHeight;
    private RobotBase.RightSlideHeight rightSlideHeight;
    private final double dblSlideGrabbingPos = 0;
    private final double dblSlideLowestPos = 0.1;
    private final double dblSlideLowPos = 0.3;
    private final double dblSlideLowMediumPos = 0.4;
    private final double dblSlideMediumPos = 0.6;
    private final double dblSlideMediumHighPos = 0.7;
    private final double dblSlideHighPos = 0.8;
    private final double dblSlideHighestPos = 1;
    public int intLeftSlidePosition = 0; //0 = GRABBING, 1 = LOWEST, 2 = LOW, 3 = LOW MEDIUM, 4 = MEDIUM, 5 = MEDIUM HIGH, 6 = HIGH, 7 = HIGHEST.
    public int intRightSlidePosition = 0; //0 = GRABBING, 1 = LOWEST, 2 = LOW, 3 = LOW MEDIUM, 4 = MEDIUM, 5 = MEDIUM HIGH, 6 = HIGH, 7 = HIGHEST.

    public SlideSubsystem(Servo leftSlideConstructor, Servo rightSlideConstructor) {
        srvSlideLeft = leftSlideConstructor;
        srvSlideRight = rightSlideConstructor;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION FOR GRABBING PIXELS
    public void slideLeftGrabbingPos() {
        srvSlideLeft.setPosition(dblSlideGrabbingPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.DOWN;
        intLeftSlidePosition = 0;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE LOWEST SET LINE
    public void slideLeftLow() {
        srvSlideLeft.setPosition(dblSlideLowPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
        intLeftSlidePosition = 2;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE MEDIUM SET LINE
    public void slideLeftMedium() {
        srvSlideLeft.setPosition(dblSlideMediumPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
        intLeftSlidePosition = 4;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE HIGH SET LINE
    public void slideLeftHigh() {
        srvSlideLeft.setPosition(dblSlideHighPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
        intLeftSlidePosition = 6;
    }

    //MOVES CURRENT LEFT SLIDE POSITION DOWN BY ONE PRESET POSITION
    public void leftSlidePositionLower() {
        switch (intLeftSlidePosition) {
            case 2:
                srvSlideLeft.setPosition(dblSlideLowestPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
                intLeftSlidePosition = 1;
                break;
            case 3:
                srvSlideLeft.setPosition(dblSlideLowPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
                intLeftSlidePosition = 2;
                break;
            case 4:
                srvSlideLeft.setPosition(dblSlideLowMediumPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
                intLeftSlidePosition = 3;
                break;
            case 5:
                srvSlideLeft.setPosition(dblSlideMediumPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
                intLeftSlidePosition = 4;
                break;
            case 6:
                srvSlideLeft.setPosition(dblSlideMediumHighPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
                intLeftSlidePosition = 5;
                break;
            case 7:
                srvSlideLeft.setPosition(dblSlideHighPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
                intLeftSlidePosition = 6;
                break;
        }
    }

    //MOVES CURRENT LEFT SLIDE POSITION UP BY ONE PRESET POSITION
    public void leftSlidePositionRaise() {
        switch (intLeftSlidePosition) {
            case 0:
                srvSlideLeft.setPosition(dblSlideLowestPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
                intLeftSlidePosition = 1;
                break;
            case 1:
                srvSlideLeft.setPosition(dblSlideLowPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
                intLeftSlidePosition = 2;
                break;
            case 2:
                srvSlideLeft.setPosition(dblSlideLowMediumPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
                intLeftSlidePosition = 3;
                break;
            case 3:
                srvSlideLeft.setPosition(dblSlideMediumPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
                intLeftSlidePosition = 4;
                break;
            case 4:
                srvSlideLeft.setPosition(dblSlideMediumHighPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
                intLeftSlidePosition = 5;
                break;
            case 5:
                srvSlideLeft.setPosition(dblSlideHighPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
                intLeftSlidePosition = 6;
                break;
            case 6:
                srvSlideLeft.setPosition(dblSlideHighestPos);
                leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
                intLeftSlidePosition = 7;
                break;
        }
    }

    //TOGGLES THE LEFT SLIDE BETWEEN GRABBING POSITION AND THE LOW POSITION
    public void leftLowToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.LOW) {
            slideLeftGrabbingPos();
        } else {
            slideLeftLow();
        }
    }

    //TOGGLES THE LEFT SLIDE BETWEEN GRABBING POSITION AND THE MEDIUM POSITION
    public void leftMediumToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.MEDIUM) {
            slideLeftGrabbingPos();
        } else {
            slideLeftMedium();
        }
    }

    //TOGGLES THE LEFT SLIDE BETWEEN GRABBING POSITION AND THE HIGH POSITION
    public void leftHighToggle() {
        if(leftSlideHeight == RobotBase.LeftSlideHeight.HIGH) {
            slideLeftGrabbingPos();
        } else {
            slideLeftHigh();
        }
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION FOR GRABBING PIXELS
    public void slideRightGrabbingPos() {
        srvSlideRight.setPosition(dblSlideHighPos);
        rightSlideHeight = RobotBase.RightSlideHeight.DOWN;
        intRightSlidePosition = 0;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE LOWEST SET LINE
    public void slideRightLow() {
        srvSlideRight.setPosition(dblSlideLowPos);
        rightSlideHeight = RobotBase.RightSlideHeight.LOW;
        intRightSlidePosition = 2;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE MEDIUM SET LINE
    public void slideRightMedium() {
        srvSlideRight.setPosition(dblSlideMediumPos);
        rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
        intRightSlidePosition = 4;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE HIGH SET LINE
    public void slideRightHigh() {
        srvSlideRight.setPosition(dblSlideHighPos);
        rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
        intRightSlidePosition = 6;
    }

    //MOVES CURRENT RIGHT SLIDE POSITION DOWN BY ONE PRESET POSITION
    public void rightSlidePositionLower() {
        switch (intRightSlidePosition) {
            case 2:
               srvSlideRight.setPosition(dblSlideLowestPos);
               rightSlideHeight = RobotBase.RightSlideHeight.LOW;
               intRightSlidePosition = 1;
               break;
            case 3:
                srvSlideRight.setPosition(dblSlideLowPos);
                rightSlideHeight = RobotBase.RightSlideHeight.LOW;
                intRightSlidePosition = 2;
                break;
            case 4:
                srvSlideRight.setPosition(dblSlideLowMediumPos);
                rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
                intRightSlidePosition = 3;
                break;
            case 5:
                srvSlideRight.setPosition(dblSlideMediumPos);
                rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
                intRightSlidePosition = 4;
                break;
            case 6:
                srvSlideRight.setPosition(dblSlideMediumHighPos);
                rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
                intRightSlidePosition = 5;
                break;
            case 7:
                srvSlideRight.setPosition(dblSlideHighPos);
                rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
                intRightSlidePosition = 6;
                break;
        }
    }

    //MOVES CURRENT RIGHT SLIDE POSITION UP BY ONE PRESET POSITION
    public void rightSlidePositionRaise() {
        switch (intRightSlidePosition) {
            case 0:
                srvSlideRight.setPosition(dblSlideLowestPos);
                intRightSlidePosition = 1;
                rightSlideHeight = RobotBase.RightSlideHeight.LOW;
                break;
            case 1:
                srvSlideRight.setPosition(dblSlideLowPos);
                intRightSlidePosition = 2;
                rightSlideHeight = RobotBase.RightSlideHeight.LOW;
                break;
            case 2:
                srvSlideRight.setPosition(dblSlideLowMediumPos);
                intRightSlidePosition = 3;
                rightSlideHeight = RobotBase.RightSlideHeight.LOW;
                break;
            case 3:
                srvSlideRight.setPosition(dblSlideMediumPos);
                intRightSlidePosition = 4;
                rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
                break;
            case 4:
                srvSlideRight.setPosition(dblSlideMediumHighPos);
                intRightSlidePosition = 5;
                rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
                break;
            case 5:
                srvSlideRight.setPosition(dblSlideHighPos);
                intRightSlidePosition = 6;
                rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
                break;
            case 6:
                srvSlideRight.setPosition(dblSlideHighestPos);
                intRightSlidePosition = 7;
                rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
                break;
        }
    }

    //TOGGLES THE RIGHT SLIDE BETWEEN GRABBING POSITION AND THE LOW POSITION
    public void rightLowToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.LOW) {
            slideRightGrabbingPos();
        } else {
            slideRightLow();
        }
    }

    //TOGGLES THE RIGHT SLIDE BETWEEN GRABBING POSITION AND THE MEDIUM POSITION
    public void rightMediumToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.MEDIUM) {
            slideRightGrabbingPos();
        } else {
            slideRightMedium();
        }
    }

    //TOGGLES THE RIGHT SLIDE BETWEEN GRABBING POSITION AND THE HIGH POSITION
    public void rightHighToggle() {
        if(rightSlideHeight == RobotBase.RightSlideHeight.HIGH) {
            slideRightGrabbingPos();
        } else {
            slideRightHigh();
        }
    }
}