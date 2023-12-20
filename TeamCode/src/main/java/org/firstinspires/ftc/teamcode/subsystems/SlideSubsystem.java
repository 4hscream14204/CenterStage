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
}