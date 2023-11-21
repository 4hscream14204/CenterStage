package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class SlideSubsystem extends SubsystemBase {

    Servo srvSlideRight;
    Servo srvSlideLeft;
    private RobotBase.LeftSlideHeight leftSlideHeight;
    private RobotBase.RightSlideHeight rightSlideHeight;
    private RobotBase.SyncSlidesMode syncSlides;
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
        leftSlideHeight = RobotBase.LeftSlideHeight.GRABBING;
        intLeftSlidePosition = 0;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION BELOW THE LOWEST SET LINE
    public void slideLeftLowest() {
        srvSlideLeft.setPosition(dblSlideLowestPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
        intLeftSlidePosition = 1;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE LOWEST SET LINE
    public void slideLeftLow() {
        srvSlideLeft.setPosition(dblSlideLowPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.LOW;
        intLeftSlidePosition = 2;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION BETWEEN THE LOWEST AND MIDDLE SET LINES
    public void slideLeftLowMedium() {
        srvSlideLeft.setPosition(dblSlideLowMediumPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
        intLeftSlidePosition = 3;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE MEDIUM SET LINE
    public void slideLeftMedium() {
        srvSlideLeft.setPosition(dblSlideMediumPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
        intLeftSlidePosition = 4;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION BETWEEN THE MIDDLE AND HIGHEST SET LINES
    public void slideLeftMediumHigh() {
        srvSlideLeft.setPosition(dblSlideMediumHighPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.MEDIUM;
        intLeftSlidePosition = 5;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ON THE HIGHEST SET LINE
    public void slideLeftHigh() {
        srvSlideLeft.setPosition(dblSlideHighPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
        intLeftSlidePosition = 6;
    }

    //PUTS THE LEFT SLIDE IN THE POSITION ABOVE THE HIGHEST SET LINE
    public void slideLeftHighest() {
        srvSlideLeft.setPosition(dblSlideHighestPos);
        leftSlideHeight = RobotBase.LeftSlideHeight.HIGH;
        intLeftSlidePosition = 7;
    }

    //MOVES CURRENT LEFT SLIDE POSITION DOWN BY ONE PRESET POSITION
    public void leftSlidePositionLower() {
        switch (intLeftSlidePosition) {
            case 2:
                slideLeftLowest();
                break;
            case 3:
                slideLeftLow();
                break;
            case 4:
                slideLeftLowMedium();
                break;
            case 5:
                slideLeftMedium();
                break;
            case 6:
                slideLeftMediumHigh();
                break;
            case 7:
                slideLeftHigh();
                break;
        }
    }

    //MOVES CURRENT LEFT SLIDE POSITION UP BY ONE PRESET POSITION
    public void leftSlidePositionRaise() {
        switch (intLeftSlidePosition) {
            case 0:
                slideLeftLowest();
                break;
            case 1:
                slideLeftLow();
                break;
            case 2:
                slideLeftLowMedium();
                break;
            case 3:
                slideLeftMedium();
                break;
            case 4:
                slideLeftMediumHigh();
                break;
            case 5:
                slideLeftHigh();
                break;
            case 6:
                slideLeftHighest();
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
        rightSlideHeight = RobotBase.RightSlideHeight.GRABBING;
        intRightSlidePosition = 0;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION BELOW THE LOWEST SET LINE
    public void slideRightLowest() {
        srvSlideRight.setPosition(dblSlideLowestPos);
        rightSlideHeight = RobotBase.RightSlideHeight.LOW;
        intRightSlidePosition = 1;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE LOWEST SET LINE
    public void slideRightLow() {
        srvSlideRight.setPosition(dblSlideLowPos);
        rightSlideHeight = RobotBase.RightSlideHeight.LOW;
        intRightSlidePosition = 2;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION BETWEEN THE LOWEST AND MIDDLE SET LINES
    public void slideRightLowMedium() {
        srvSlideRight.setPosition(dblSlideLowMediumPos);
        rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
        intRightSlidePosition = 3;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE MEDIUM SET LINE
    public void slideRightMedium() {
        srvSlideRight.setPosition(dblSlideMediumPos);
        rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
        intRightSlidePosition = 4;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION BETWEEN THE MIDDLE AND HIGHEST SET LINES
    public void slideRightMediumHigh() {
        srvSlideRight.setPosition(dblSlideMediumHighPos);
        rightSlideHeight = RobotBase.RightSlideHeight.MEDIUM;
        intRightSlidePosition = 5;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ON THE HIGHEST SET LINE
    public void slideRightHigh() {
        srvSlideRight.setPosition(dblSlideHighPos);
        rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
        intRightSlidePosition = 6;
    }

    //PUTS THE RIGHT SLIDE IN THE POSITION ABOVE THE HIGHEST SET LINE
    public void slideRightHighest() {
        srvSlideRight.setPosition(dblSlideHighestPos);
        intRightSlidePosition = 7;
        rightSlideHeight = RobotBase.RightSlideHeight.HIGH;
    }

    //MOVES CURRENT RIGHT SLIDE POSITION DOWN BY ONE PRESET POSITION
    public void rightSlidePositionLower() {
        switch (intRightSlidePosition) {
            case 2:
               slideRightLowest();
               break;
            case 3:
                slideRightLow();
                break;
            case 4:
                slideRightLowMedium();
                break;
            case 5:
                slideRightMedium();
                break;
            case 6:
                slideRightMediumHigh();
                break;
            case 7:
                slideRightHigh();
                break;
        }
    }

    //MOVES CURRENT RIGHT SLIDE POSITION UP BY ONE PRESET POSITION
    public void rightSlidePositionRaise() {
        switch (intRightSlidePosition) {
            case 0:
                slideRightLowest();
                break;
            case 1:
                slideRightLow();
                break;
            case 2:
                slideRightLowMedium();
                break;
            case 3:
                slideRightMedium();
                break;
            case 4:
                slideRightMediumHigh();
                break;
            case 5:
                slideRightHigh();
                break;
            case 6:
                slideRightHighest();
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
    //SYNC SLIDES MODE VERSIONS

    //THIS IS THE OPERATION FOR SYNCING THE SLIDES AND UNSYNCING THE SLIDES
    public void syncSlidesToggle() {
        switch(syncSlides) {
            case OFF:
            switch (intRightSlidePosition) {
                case 0:
                    slideLeftGrabbingPos();
                    syncSlides = RobotBase.SyncSlidesMode.ON;
                    break;
                case 1:
                    slideLeftLowest();
                    syncSlides = RobotBase.SyncSlidesMode.ON;
                    break;
                case 2:
                    slideLeftLow();
                    syncSlides = RobotBase.SyncSlidesMode.ON;
                    break;
                case 3:
                    slideLeftLowMedium();
                    syncSlides = RobotBase.SyncSlidesMode.ON;
                    break;
                case 4:
                    slideLeftMedium();
                    syncSlides = RobotBase.SyncSlidesMode.ON;
                    break;
                case 5:
                    slideLeftMediumHigh();
                    syncSlides = RobotBase.SyncSlidesMode.ON;
                    break;
                case 6:
                    slideLeftHigh();
                    syncSlides = RobotBase.SyncSlidesMode.ON;
                    break;
                case 7:
                    slideLeftHighest();
                    syncSlides = RobotBase.SyncSlidesMode.ON;
                    break;
            }
            break;
            case ON:
                syncSlides = RobotBase.SyncSlidesMode.OFF;
                break;
        }
    }

    //PUTS BOTH SLIDES IN THE POSITION FOR GRABBING PIXELS
    public void slideGrabbingPos() {
        slideLeftGrabbingPos();
        slideRightGrabbingPos();
    }

    //PUTS BOTH SLIDES IN THE POSITION BELOW THE LOWEST SET LINE
    public void slideLowestPos() {
        slideLeftLowest();
        slideRightLowest();
    }

    //PUTS BOTH SLIDES IN THE POSITION ON THE LOWEST SET LINE
    public void slideLowPos() {
        slideLeftLow();
        slideRightLow();
    }

    //PUTS BOTH SLIDE IN THE POSITION BETWEEN THE LOWEST AND MIDDLE SET LINES
    public void slideLowMediumPos() {
        slideLeftLowMedium();
        slideRightLowMedium();
    }

    //PUTS BOTH SLIDES IN THE POSITION ON THE MEDIUM SET LINE
    public void slideMediumPos() {
        slideLeftMedium();
        slideRightMedium();
    }

    //PUTS BOTH SLIDE IN THE POSITION BETWEEN THE MIDDLE AND HIGHEST SET LINES
    public void slideMediumHighPos() {
        slideLeftMediumHigh();
        slideRightMediumHigh();
    }

    //PUTS BOTH SLIDES IN THE POSITION ON THE HIGHEST SET LINE
    public void slideHighPos() {
        slideLeftHigh();
        slideRightHigh();
    }

    //PUTS BOTH SLIDES IN THE POSITION ABOVE THE HIGHEST SET LINE
    public void slideHighestPos() {
        slideLeftHighest();
        slideRightHighest();
    }
}