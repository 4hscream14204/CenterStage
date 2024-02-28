package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.LogitechCameraSubsystemBetter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.subsystems.LogitechCameraSubsystemBetter;
import android.util.Size;

@TeleOp(name="LogitechDetect")
public class LogitechDetectionTest extends OpMode {

    public RobotBase robotBase;
    private LogitechCameraSubsystemBetter visionProcesser;
    private VisionPortal visionPortal;

    @Override
    public void init() {
      //  robotBase = new RobotBase(hardwareMap);

        visionProcesser = new LogitechCameraSubsystemBetter(RobotBase.StartPosition.LEFT);
      //  visionPortal = VisionPortal.easyCreateWithDefaults(
               // hardwareMap.get(WebcamName.class, "Webcam1"),visionProcesser);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(visionProcesser)
                .setCameraResolution(new Size(864, 480))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


/*
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }

 */


        //visionPortal = builder.build();

    }

    public void start (){
        visionPortal.stopStreaming();
    }

@Override
    public void loop(){

    }



}

