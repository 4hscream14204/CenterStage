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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.commands.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.DropOffPositionLowCommandGrp;
import org.firstinspires.ftc.teamcode.commands.GrabAndWristEscapeCommandGrp;
import org.firstinspires.ftc.teamcode.commands.UniversalGrabbingPosCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LogitechCameraSubsystemBetter;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Right CRI")
public class BlueRightCRI extends OpMode {

    public RobotBase robotBase;
    enum CurrentRouteState {
        SPIKE,
        CROSS,
        DROP,
        PARKING,
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
    private TrajectorySequence LeftBackDropOff;
    private TrajectorySequence MiddleBackDropOff;
    private TrajectorySequence RightBackDropOff;
    private TrajectorySequence spikeLocation;
    private TrajectorySequence parkLocation;
    private TrajectorySequence crossing;
    private TrajectorySequence backDropOff;
    private TrajectorySequence InnerPark;
    private TrajectorySequence OuterPark;
    private CurrentRouteState currentRouteState;


    public void init() {
        CommandScheduler.getInstance().reset();
        autoChassisController = new GamepadEx(gamepad1);
        robotBase = new RobotBase(hardwareMap);
        robotBase.parkSide = RobotBase.ParkSide.INNER;
        //robotBase.crossSide = RobotBase.CrossSide.INSIDE;
        //crossing = InnerCross;
        robotBase.alliance = RobotBase.Alliance.BLUE;
        robotBase.spikeLocation = RobotBase.SpikeLocation.LEFTSPIKE;
        robotBase.backDropOff = RobotBase.BackDropOff.LEFTDROP;
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
        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);

        LeftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-79.5, 31.00), Math.toRadians(315.00))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-88, 58, Math.toRadians(0.00)), Math.toRadians(180.00))
                .build();

        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-86.00, 33.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-88, 58, Math.toRadians(0.00)), Math.toRadians(180.00))
                .build();

        RightSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-91.00, 38.00, Math.toRadians(225.00)), Math.toRadians(225.00))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-88, 58, Math.toRadians(0.00)), Math.toRadians(180.00))
                .build();

        //Temporarily not using Innercross
     /*   InnerCross = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-96, 48, Math.toRadians(180.00)))
                .splineToLinearHeading(new Pose2d(-98, 12, Math.toRadians(180.00)), Math.toRadians(180.00))
                //Code for picking up needed
                .splineToConstantHeading(new Vector2d(12.00, 12.00), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(45,36), Math.toRadians(180.00))
                .build(); */

        OuterCross = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-88, 58, Math.toRadians(0.00)))
                .splineToLinearHeading(new Pose2d(-80,58, Math.toRadians(0.00)), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(20,58), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(45,33), Math.toRadians(270.00))
                .build();

        LeftBackDropOff = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45, 33, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                 robotBase.leftClawSubsystem,
                robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(45,34.00), Math.toRadians(90.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .lineToConstantHeading(new Vector2d(50,34.00))
                .waitSeconds(0.5)
                .addDisplacementMarker( () -> {
                    robotBase.leftClawSubsystem.clawOpen();
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(45,34.00))
                .addTemporalMarker( () -> {
                    robotBase.leftWristSubsystem.wristPickup();
                })
                .lineToConstantHeading(new Vector2d(45,35))
                .addTemporalMarker( () -> {
                    robotBase.armSubsystem.armGrabbingPosition();
                })
                .build();

        MiddleBackDropOff = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45, 33, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(45,34.00), Math.toRadians(90.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .lineToConstantHeading(new Vector2d(50,34.00))
                .waitSeconds(0.5)
                .addDisplacementMarker( () -> {
                    robotBase.leftClawSubsystem.clawOpen();
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(45,34.00))
                .addTemporalMarker( () -> {
                    robotBase.leftWristSubsystem.wristPickup();
                })
                .lineToConstantHeading(new Vector2d(45,35))
                .addTemporalMarker( () -> {
                    robotBase.armSubsystem.armGrabbingPosition();
                })
                .build();

        RightBackDropOff = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45, 33, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(48, 30), Math.toRadians(180.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(46.00, 28.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(1)
                .addDisplacementMarker( () -> {
                    robotBase.leftClawSubsystem.clawOpen();
                })
                .lineTo(new Vector2d(40, 36))
                .waitSeconds(1)
                .addDisplacementMarker( () -> {
                    robotBase.leftWristSubsystem.wristPickup();
                })
                .waitSeconds(0.5)
                .addDisplacementMarker( () -> {
                    robotBase.armSubsystem.armGrabbingPosition();
                })
                .waitSeconds(1)
                .build();

        OuterPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 35.00, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(60.00, 60.00), Math.toRadians(0.00))
                .build();

        InnerPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 35.00, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(50.00, 10.00), Math.toRadians(0.00))
                .build();

        parkLocation = InnerPark;
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
       /* if(autoChassisController.wasJustPressed((GamepadKeys.Button.X))) {
            if (robotBase.crossSide == RobotBase.CrossSide.INSIDE) {
                robotBase.crossSide = RobotBase.CrossSide.OUTSIDE;
                crossing = OuterCross;
            } else {
                robotBase.crossSide = RobotBase.CrossSide.INSIDE;
                crossing = InnerCross;
            }
        } */

        //Detection for spike trajectory and for backdrop drop off
        robotBase.propPosition = visionProcesser.getLocation();
        if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.spikeLocation = RobotBase.SpikeLocation.MIDDLESPIKE;
            spikeLocation = MiddleSpike;
            backDropOff = MiddleBackDropOff;
        } else if(robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.spikeLocation = RobotBase.SpikeLocation.RIGHTSPIKE;
            spikeLocation = RightSpike;
            backDropOff = RightBackDropOff;
        } else {
            robotBase.spikeLocation = RobotBase.SpikeLocation.LEFTSPIKE;
            spikeLocation = LeftSpike;
            backDropOff = LeftBackDropOff;
        }


        telemetry.addData("InitLoop", "true");
        telemetry.addData("Detection", (robotBase.propPosition));
        telemetry.addLine("Y = Park Side");
        //telemetry.addData("Cross Side", (robotBase.crossSide));
        telemetry.addData("Park Side", (robotBase.parkSide));
        telemetry.update();
    }
    public void start () {
        visionPortal.stopStreaming();
        /* if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(MiddleSpike);
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(crossing);
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(MiddleBackDropOff);
        } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(RightSpike);
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(crossing);
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(RightBackDropOff);
        } else {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(LeftSpike);
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(crossing);
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(LeftBackDropOff);
            } */
        currentRouteState = CurrentRouteState.SPIKE;
        robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(spikeLocation);
    }

    public void loop () {


        switch (currentRouteState) {
            case SPIKE:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = CurrentRouteState.CROSS;
                    robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(OuterCross);
                }
            case CROSS:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = CurrentRouteState.DROP;
                    robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(backDropOff);
                }
            case DROP:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = CurrentRouteState.PARKING;
                    robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(parkLocation);
                }
        }
        telemetry.addData("Current Trajectory", currentRouteState);
        robotBase.mecanumDriveSubsystem.update();
        CommandScheduler.getInstance().run();
    }
    public void stop (){
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
        DataStorageSubsystem.alliance = robotBase.alliance.BLUE;
    }
}
