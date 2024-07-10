package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LogitechCameraSubsystemBetter;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Right CRI")
public class BlueRightCRI extends OpMode {

    public RobotBase robotBase;
    enum CurrentRouteState {
        TRAJECTORY_1,
        PARKING
    }
    public GamepadEx autoChassisController;
    public Pose2d startPose;
    private LogitechCameraSubsystemBetter visionProcesser;
    private VisionPortal visionPortal;
    private TrajectorySequence LeftSpike;
    private TrajectorySequence MiddleSpike;
    private TrajectorySequence RightSpike;
    private TrajectorySequence InnerCross;
    private TrajectorySequence OuterCross;
    private TrajectorySequence Leftbackdropoff;
    private TrajectorySequence Middlebackdropoff;
    private TrajectorySequence Rightbackdropoff;
    private TrajectorySequence parkLocation;
    private TrajectorySequence crossing;
    private TrajectorySequence InnerPark;
    private TrajectorySequence OuterPark;
    private CurrentRouteState currentRouteState;


    public void init() {
        CommandScheduler.getInstance().reset();
        autoChassisController = new GamepadEx(gamepad1);
        robotBase = new RobotBase(hardwareMap);
        robotBase.parkSide = RobotBase.ParkSide.INNER;
        robotBase.alliance = RobotBase.Alliance.BLUE;
        visionProcesser = new LogitechCameraSubsystemBetter(RobotBase.StartPosition.RIGHT);
        robotBase.leftClawSubsystem.clawClose();
        robotBase.leftWristSubsystem.wristEscape();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(visionProcesser)
                .setCameraResolution(new Size(864, 480))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        startPose = new Pose2d(-88.00, 61.00, Math.toRadians(270.00));

        LeftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d( -72, 30, Math.toRadians(315.00)), Math.toRadians(315.00))
                .splineToLinearHeading(new Pose2d(-96, 48, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();

        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-84, 25, Math.toRadians(270.00)), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(-96, 48, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();

        RightSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-95, 30, Math.toRadians(225.00)), Math.toRadians(225.00))
                .splineToLinearHeading(new Pose2d(-96, 48, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();

        InnerCross = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-96, 48, Math.toRadians(180.00)))
                .splineToLinearHeading(new Pose2d(-198,46, Math.toRadians(225.00)), Math.toRadians(225.00))
                //Code for picking up needed
                .splineToConstantHeading(new Vector2d(12.00, 12.00), Math.toRadians(180.00))
                .build();

        OuterCross = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-96, 48, Math.toRadians(180.00)))
                .splineToLinearHeading(new Pose2d(-204, 12, Math.toRadians(180.00)), Math.toRadians(180.00))
                //Code for picking up needed
                .splineToLinearHeading(new Pose2d(-84,60, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();

        Leftbackdropoff = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(180.00)))

                .build();

        Middlebackdropoff = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(180.00)))

                .build();

        Rightbackdropoff = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(180.00)))

                .build();

        OuterPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .lineTo(new Vector2d(45.00, 62.00))
                .lineTo(new Vector2d(55.00, 62.00))
                .lineTo(new Vector2d(45.00, 62.00))
                .build();

        InnerPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .lineTo(new Vector2d(45.00, 12.00))
                .lineTo(new Vector2d(55.00, 12.00))
                .lineTo(new Vector2d(45.00, 12.00))
                .build();
    }
    public void init_loop() {
        autoChassisController.readButtons();
        //Button press to change parking
        if (autoChassisController.wasJustPressed((GamepadKeys.Button.Y))) {
            if (robotBase.parkSide == RobotBase.ParkSide.INNER) {
                robotBase.parkSide = RobotBase.ParkSide.OUTER;
                parkLocation = OuterPark;
            } else {
                robotBase.parkSide = RobotBase.ParkSide.INNER;
                parkLocation = InnerPark;
            }
        }
        //Button press to change crosside
        if(autoChassisController.wasJustPressed((GamepadKeys.Button.X))) {
            if (robotBase.crossSide == RobotBase.CrossSide.INSIDE) {
                robotBase.crossSide = RobotBase.CrossSide.OUTSIDE;
                crossing = OuterCross;
            } else {
                robotBase.crossSide = RobotBase.CrossSide.INSIDE;
                crossing = InnerCross;
            }
        }
        robotBase.propPosition = visionProcesser.getLocation();

        telemetry.addData("InitLoop", "true");
        telemetry.addData("Detection", (robotBase.propPosition));
        telemetry.addData("Park Side", (robotBase.parkSide));
        telemetry.update();
    }
    public void start () {
        if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(MiddleSpike);
        } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(RightSpike);
        } else {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(LeftSpike);
        }
        currentRouteState = BlueRightCRI.CurrentRouteState.TRAJECTORY_1;
    }
    public void loop () {
        switch (currentRouteState) {
            case TRAJECTORY_1:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = BlueRightCRI.CurrentRouteState.PARKING;
                    robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(parkLocation);
                }
        }
        robotBase.mecanumDriveSubsystem.update();
    }
    public void stop (){
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
        DataStorageSubsystem.alliance = robotBase.alliance.BLUE;
    }
}
