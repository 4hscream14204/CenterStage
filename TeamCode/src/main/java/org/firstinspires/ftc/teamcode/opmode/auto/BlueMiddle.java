package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.DropOffPositionLowCommandGrp;
import org.firstinspires.ftc.teamcode.commands.GrabAndWristEscapeCommandGrp;
import org.firstinspires.ftc.teamcode.commands.UniversalGrabbingPosCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LogitechCameraSubsystemBetter;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "BlueMiddle")
public class BlueMiddle extends OpMode {
    public RobotBase robotBase;

    private enum CurrentRouteState {
        TRAJECTORY_1,
        PARKING,
        STACK,
        SPIKE,
        CROSS
    }


    public TrajectorySequence LeftSpike;
    public TrajectorySequence MiddleSpike;
    public TrajectorySequence RightSpike;

    private TrajectorySequence InnerPark;
    private TrajectorySequence MiddlePark;
    private TrajectorySequence OuterPark;
    private TrajectorySequence StackPickup;
    private TrajectorySequence parkLocation;
    private TrajectorySequence InnerCross;
    private TrajectorySequence OuterCross;
    private TrajectorySequence cross;
    private TrajectorySequence InnerStackPickup;
    private TrajectorySequence OuterStackPickup;

    public Pose2d startPose;

    public Pose2d stackPose;

    public GamepadEx autoChassisController;
    private CurrentRouteState currentRouteState;

    private LogitechCameraSubsystemBetter visionProcesser;
    private VisionPortal visionPortal;

    @Override
    public void init(){
        CommandScheduler.getInstance().reset();
        autoChassisController = new GamepadEx(gamepad1);
        robotBase = new RobotBase(hardwareMap);
        robotBase.parkSide = RobotBase.ParkSide.INNER;
        robotBase.crossSide = RobotBase.CrossSide.INSIDE;
        robotBase.alliance = RobotBase.Alliance.BLUE;
        robotBase.startPosition = RobotBase.StartPosition.RIGHT;
        visionProcesser = new LogitechCameraSubsystemBetter(RobotBase.StartPosition.LEFT);
        robotBase.leftClawSubsystem.clawClose();
        robotBase.leftWristSubsystem.wristEscape();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(visionProcesser)
                .setCameraResolution(new Size(864, 480))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        startPose = new Pose2d(-30, 63.3, Math.toRadians(270.00));

        stackPose = new Pose2d(-56.66, 52.77, Math.toRadians(270));

        LeftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .waitSeconds(15)
                .splineTo(new Vector2d(-30.00, 36.00), Math.toRadians(-45))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-30.50, 60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(45.00, 36.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToConstantHeading(new Vector2d(51.00, 36.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(45.00, 36.00))

                //USING COMMANDS
                /*.waitSeconds(0)//15
                .splineTo(new Vector2d(-27.00, 36.00), Math.toRadians(-45))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-36.0, 36.00, Math.toRadians(0.00)), Math.toRadians(0.00))*/
                /*
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(45.00, 36.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToConstantHeading(new Vector2d(51.00, 36.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(40.00, 36.00))
                */
                .build();


        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .waitSeconds(15)
                .splineTo(new Vector2d(-36.00, 35.00), Math.toRadians(270.00))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-30.00, 60.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToConstantHeading(new Vector2d(45.00, 33.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(51.00, 33.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(45.00, 36.00))

                /*.splineTo(new Vector2d(-35.96, 33.18), Math.toRadians(-90.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(-34, 45), Math.toRadians(-90))
                .splineTo(new Vector2d(-34, 48), Math.toRadians(0))*/
                /*.waitSeconds(0.25)
                .splineTo(new Vector2d(-48.66, 52.77), Math.toRadians(180.95))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.rightClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(
                        ()->robotBase.intakeSubsystem.intake(-1)
                )))
                .setReversed(true)*/
                .build();


        RightSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                .waitSeconds(15)
                .splineTo(new Vector2d(-40, 39.11), Math.toRadians(225.00))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-30.00, 60.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToConstantHeading(new Vector2d(45.00, 26.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(51.00, 26.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(45.00, 36.00))

                /*.waitSeconds(15)
                .splineTo(new Vector2d(-40, 39.11), Math.toRadians(225.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-34, 48), Math.toRadians(220))*/
                /*.splineToSplineHeading(new Pose2d(-36.00, 60.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToConstantHeading(new Vector2d(45.00, 26.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(51.00, 26.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(40.00, 36.00))*/
                .build();




        OuterPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(45,58), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(62,58), Math.toRadians(0.00))
                .build();

        MiddlePark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(50, 36), Math.toRadians(0))
                .build();

        InnerPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45, 36, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(28.64, 53.98), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(38.16, 34.94), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(49.15, 18.38), Math.toRadians(0))
                .build();


        /*InnerCross = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-34, 48, Math.toRadians(0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24, 58), Math.toRadians(0))
                //Code for picking up needed
                .splineToConstantHeading(new Vector2d(24.00, 18.00), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45,36), Math.toRadians(0))
                .build();

        OuterCross = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-34, 48, Math.toRadians(-90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-32.87, 12), Math.toRadians(-90))
                //Code for picking up needed
                .splineToLinearHeading(new Pose2d(25.56, 12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(49.15, 35.38), Math.toRadians(0))
                .build();*/

        /*InnerStackPickup = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(25, 15),  Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-90, 15), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-108, 15), Math.toRadians(180))
                .addDisplacementMarker(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.rakeSubsystem.rakePosition(0.8))
                ))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-95, 15), Math.toRadians(180))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-98, 15), Math.toRadians(0.00))
                .addDisplacementMarker(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.rakeSubsystem.rakePosition( 0))
                ))
                .splineToConstantHeading(new Vector2d(25, 15), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(45, 36),  Math.toRadians(90))
                .build();

        OuterStackPickup = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(25, 63),  Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-98, 63), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-108, 50, Math.toRadians(45)), Math.toRadians(180))
                .addDisplacementMarker(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.rakeSubsystem.rakePosition(0.8))
                ))
                .setReversed(false)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-98, 63), Math.toRadians(0.00))
                .addDisplacementMarker(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.rakeSubsystem.rakePosition( 0))
                ))
                .splineToConstantHeading(new Vector2d(25, 61), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(45, 36),  Math.toRadians(270))
                .build();*/

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);

        /*StackPickup = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-48.66, 52.77, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-48.15, 20.17), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48.41, 12.32), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-54.52, 11.30), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-56.52, 11.30), Math.toRadians(0))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(()-> robotBase.rakeSubsystem.rakePosition(1))))
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(-50.52, 11.30), Math.toRadians(0))
                .build();*/

        //TOUCH SENSOR CODE
        //RIGHT TOUCH SENSOR
        new Trigger(()-> robotBase.rightTouchSensorSubsystem.pixelInIntake())
                .whileActiveContinuous(()->CommandScheduler.getInstance().schedule(
                        new ParallelCommandGroup(
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.rightWristSubsystem, robotBase.rightClawSubsystem, robotBase.armSubsystem),
                                new InstantCommand(()-> robotBase.rightLightsSubsystem.redLightOn())
                        )
                ));
        //LEFT TOUCH SENSOR
        new Trigger(()-> robotBase.leftTouchSensorSubsystem.pixelInIntake())
                .whileActiveContinuous(()->CommandScheduler.getInstance().schedule(
                        new ParallelCommandGroup(
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.leftWristSubsystem, robotBase.leftClawSubsystem, robotBase.armSubsystem),
                                new InstantCommand(()-> robotBase.leftLightsSubsystem.redLightOn())
                        )
                ));

        parkLocation = InnerPark;
    }
    @Override
    public void init_loop() {
        autoChassisController.readButtons();
        if (autoChassisController.wasJustPressed((GamepadKeys.Button.Y))) {
            if (robotBase.parkSide == RobotBase.ParkSide.INNER) {
                robotBase.parkSide = RobotBase.ParkSide.OUTER;
                parkLocation = OuterPark;
            } else if (robotBase.parkSide == RobotBase.ParkSide.OUTER) {
                robotBase.parkSide = RobotBase.ParkSide.MIDDLE;
                parkLocation = MiddlePark;
            } else {
                robotBase.parkSide = RobotBase.ParkSide.INNER;
                parkLocation = InnerPark;
            }
        }

        if(autoChassisController.wasJustPressed((GamepadKeys.Button.X))) {
            if (robotBase.crossSide == RobotBase.CrossSide.INSIDE) {
                robotBase.crossSide = RobotBase.CrossSide.OUTSIDE;
                cross = OuterCross;
            } else {
                robotBase.crossSide = RobotBase.CrossSide.INSIDE;
                cross = InnerCross;
            }
        }
        //robotBase.propPosition = robotBase.huskyLensSubsystem.getLocation(robotBase.alliance, robotBase.startPosition);
        robotBase.propPosition = visionProcesser.getLocation();
        telemetry.addData("InitLoop","true");
        telemetry.addData("Detection",(robotBase.propPosition));
        telemetry.addData("Park Side", (robotBase.parkSide));
        telemetry.addData("Cross Side", (robotBase.crossSide));
        telemetry.update();
    }
    @Override
    public void start () {
        visionPortal.stopStreaming();
        if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(MiddleSpike);
            //robotBase.grabber.drop();
            //robotBase.mecanumDrive.followTrajectorySequence(RedRightCenterInner2);
        } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(RightSpike);
        } else {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(LeftSpike);
        }
        currentRouteState = CurrentRouteState.TRAJECTORY_1;



    }
    @Override
    public void loop () {


        switch (currentRouteState) {
            case TRAJECTORY_1:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = CurrentRouteState.CROSS;
                    robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(cross);
                }
            /*case CROSS:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = CurrentRouteState.STACK;
                    robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(InnerStackPickup);
                } */
            case CROSS:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = CurrentRouteState.PARKING;
                    robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(parkLocation);
                }
        }





        robotBase.mecanumDriveSubsystem.update();
        CommandScheduler.getInstance().run();
        telemetry.addData("Current Trajectory", currentRouteState);
    }
    @Override
    public void stop () {
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
        DataStorageSubsystem.alliance = robotBase.alliance.BLUE;
    }
}
//BlueMiddleSpikeExtraCode
//Old Stack Pickup code
    /*.splineTo(new Vector2d(-35.96, 33.18), Math.toRadians(-90.00))
                .waitSeconds(0.25)
                .splineTo(new Vector2d(-37.14, 52.81), Math.toRadians(90.00))
                .waitSeconds(0.25)
                .splineTo(new Vector2d(-48.66, 52.77), Math.toRadians(180.95))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.rightClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(
                        ()->robotBase.intakeSubsystem.intake(-1)
                )))
                .setReversed(true)
                .build();*/

        /*.waitSeconds(15)
                .splineTo(new Vector2d(-36.00, 35.00), Math.toRadians(270.00))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-36.00, 60.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToConstantHeading(new Vector2d(45.00, 33.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(51.00, 33.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(40.00, 36.00))*/

                /*.waitSeconds(10)
                .splineToLinearHeading(new Pose2d(-36.00, 34.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .lineToLinearHeading(new Pose2d(-34.5, 59, Math.toRadians(315.00)))
                .splineToSplineHeading(new Pose2d(-5.00, 38.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(45.00, 36.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(1.5)
                .addTemporalMarker(15, () -> { robotBase.armSubsystem.armDropOffLowestPos();})
                .addTemporalMarker(15.5, () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                .lineTo(new Vector2d(53,33))
                .waitSeconds(2.5)
                .addTemporalMarker(18.5, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .lineTo(new Vector2d(43, 28))
                .addTemporalMarker(20, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .addTemporalMarker(20.5, () -> { robotBase.armSubsystem.armGrabbingPosition();})
                .build();*/

//BlueMiddleRightSpikeExtraCode
/*.splineToLinearHeading(new Pose2d(-50.00, 40.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(-57.00, 45.00), Math.toRadians(270.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(-57.00, 20.00), Math.toRadians(270.00))
                .splineToLinearHeading(new Pose2d(-57.00, 12.00, Math.toRadians(180.00)), Math.toRadians(270.00))*/
//INTAKE
        /*.splineTo(new Vector2d(-40.00, 39.00), Math.toRadians(225.00))
                .waitSeconds(0.25)
                .splineTo(new Vector2d(-48.66, 52.77), Math.toRadians(0))
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.rightClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new InstantCommand(
                        ()->robotBase.intakeSubsystem.intake(-1)
                )))
                .setReversed(true)
                .build();*/
//BlueMiddleLeftSpikeExtraCode
//OLD COMMANDS
                /*.waitSeconds(10)
                .splineToLinearHeading(new Pose2d(-28.00, 39.00, Math.toRadians(315.00)), Math.toRadians(315.00))
                .lineToSplineHeading(new Pose2d(-40.00, 50.00, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-20.00, 12.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .splineTo(new Vector2d(36.04, 24.19), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, 40.5),Math.toRadians(0.00))
                .waitSeconds(2.5)
                .addTemporalMarker(16.5, () -> { robotBase.armSubsystem.armDropOffLowestPos();})
                .addTemporalMarker(17, () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                .lineTo(new Vector2d(53, 40.5))
                .waitSeconds(1)
                .lineTo(new Vector2d(43, 28))
                .addTemporalMarker(21.5, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .addTemporalMarker(22, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .addTemporalMarker(22.5, () -> { robotBase.armSubsystem.armGrabbingPosition();})
                .build();*/
 /*.waitSeconds(10)
                .splineToLinearHeading(new Pose2d(-28.00, 39.00, Math.toRadians(315.00)), Math.toRadians(315.00))
                .lineToSplineHeading(new Pose2d(-40.00, 50.00, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-20.00, 12.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .splineTo(new Vector2d(36.04, 24.19), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, 40.5),Math.toRadians(0.00))
                .waitSeconds(2.5)
                .addTemporalMarker(16.5, () -> { robotBase.armSubsystem.armDropOffLowestPos();})
                .addTemporalMarker(17, () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                .lineTo(new Vector2d(53, 40.5))
                .waitSeconds(1)
                .lineTo(new Vector2d(43, 28))
                .addTemporalMarker(21.5, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .addTemporalMarker(22, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .addTemporalMarker(22.5, () -> { robotBase.armSubsystem.armGrabbingPosition();})
                .build();

                 */