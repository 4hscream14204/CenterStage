package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

@Autonomous(name = "CameraTest")
public class AutoCameraTest extends LinearOpMode{

    private OpenCvCamera camera;

    private String webcamName = "Webcam 1";

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    @Override

    public void runOpMode() throws InterruptedException {

        initAprilTag();

        int numOfAprilTags = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(36, 12, Math.toRadians(120));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
        //.splineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90)))

                .splineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)), Math.toRadians(120))

                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .setReversed(true)

                        .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())

                .setReversed(true)
                .lineToLinearHeading(new Pose2d(12,36, Math.toRadians(90)))
                        .build();

        waitForStart();

        resetRuntime();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        numOfAprilTags =  currentDetections.size();

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);


        if (numOfAprilTags == 1) {
            drive.followTrajectorySequence(traj2);
        }
        else if (numOfAprilTags < 1 ){
            drive.followTrajectorySequence(traj3);
        }



            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    telemetryAprilTag();

                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    // Save CPU resources; can resume streaming when needed.
                    if (gamepad1.dpad_down) {
                        visionPortal.stopStreaming();
                    } else if (gamepad1.dpad_up) {
                        visionPortal.resumeStreaming();
                    }

                    // Share the CPU.
                    sleep(20);
                }
            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        }   // end method runOpMode()

        /**
         * Initialize the AprilTag processor.
         */
        private void initAprilTag() {

            // Create the AprilTag processor the easy way.
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();

            // Create the vision portal the easy way.
            if (USE_WEBCAM) {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
            } else {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        BuiltinCameraDirection.BACK, aprilTag);
            }

        }   // end method initAprilTag()


        /**
         * Add telemetry about AprilTag detections.
         */
        private void telemetryAprilTag(){

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");


        }
        }



