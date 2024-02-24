package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Autonomous(name = "RedLeft")
public class RedLeft extends OpMode {
    public RobotBase robotBase;
    private TrajectorySequence RedLeftPark;
    public Pose2d startPose;
    private TrajectorySequence LeftSpike;
    private TrajectorySequence MiddleSpike;
    private TrajectorySequence RightSpike;

    private TrajectorySequence InnerPark;
    private TrajectorySequence OuterPark;
    private TrajectorySequence parkLocation;

    private enum CurrentRouteState {
        TRAJECTORY_1,
        PARKING
    }

    public GamepadEx autoChassisController;
    private CurrentRouteState currentRouteState;

    @Override
    public void init(){
            CommandScheduler.getInstance().reset();
            autoChassisController = new GamepadEx(gamepad1);
            robotBase = new RobotBase(hardwareMap);
            robotBase.parkSide = RobotBase.ParkSide.INNER;
            robotBase.alliance = RobotBase.Alliance.RED;
            robotBase.startPosition = RobotBase.StartPosition.LEFT;
            robotBase.leftClawSubsystem.clawClose();
            robotBase.leftWristSubsystem.wristEscape();
            startPose = new Pose2d(-38.35, -63.3, Math.toRadians(90.00));
        LeftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-41, -63.3, Math.toRadians(90.00)))
                .waitSeconds(15)
                .splineTo(new Vector2d(-40, -39.11), Math.toRadians(135.00))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-36.00, -60.00), Math.toRadians(360))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(12.00, -60.00), Math.toRadians(360))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToConstantHeading(new Vector2d(45.00, -30.00), Math.toRadians(360.00))
                .splineToConstantHeading(new Vector2d(51.00, -30.00), Math.toRadians(360.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(45.00, -36.00))
                .build();

        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-41, -63.3, Math.toRadians(90.00)))
                .waitSeconds(15)
                .splineTo(new Vector2d(-36.00, -35.00), Math.toRadians(90.00))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-36.00, -60.00), Math.toRadians(360.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                        robotBase.leftClawSubsystem,
                        robotBase.armSubsystem)))
                .splineToConstantHeading(new Vector2d(12.00, -60.00), Math.toRadians(360.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToConstantHeading(new Vector2d(45.00, -40.00), Math.toRadians(360.00))
                .splineToConstantHeading(new Vector2d(52.00, -40.00), Math.toRadians(360.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.leftClawSubsystem)))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(45.00, -36.00))
                    .build();

                    /*.waitSeconds(7)
                    .splineToLinearHeading(new Pose2d(-36.00, -34.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                    .splineToLinearHeading(new Pose2d(-34, -43, Math.toRadians(45.00)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-5.00, -38.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                    .addTemporalMarker(16, () -> { robotBase.armSubsystem.armDropOffLowestPos();})
                    .addTemporalMarker(16.5, () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                    .splineTo(new Vector2d(40,-36.5), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(53,-36.5), Math.toRadians(0))
                    .waitSeconds(0.5)
                    .splineTo(new Vector2d(43, -36.5), Math.toRadians(0))
                    .addTemporalMarker(18, () -> { robotBase.leftClawSubsystem.clawOpen();})
                    .addTemporalMarker(18.5, () -> { robotBase.leftWristSubsystem.wristPickup();})
                    .addTemporalMarker(19, () -> { robotBase.armSubsystem.armGrabbingPosition();})
                    .build();*/



            RightSpike  = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                    .waitSeconds(15)
                    .splineTo(new Vector2d(-30.00, -36.00), Math.toRadians(405.00))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(-36.00, -60.00, Math.toRadians(360.00)), Math.toRadians(360.00))
                    .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                            robotBase.leftClawSubsystem,
                            robotBase.armSubsystem)))
                    .splineToConstantHeading(new Vector2d(12.00, -60.00), Math.toRadians(360.00))
                    .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                            robotBase.armSubsystem,
                            robotBase.leftWristSubsystem,
                            robotBase.intakeSubsystem,
                            RobotBase.SlideHeight.LOWEST)))
                    .splineToConstantHeading(new Vector2d(45.00, -45.00), Math.toRadians(360.00))
                    .splineTo(new Vector2d(53.00, -45.00), Math.toRadians(360.00))
                    .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                            robotBase.leftClawSubsystem)))
                    .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                    .waitSeconds(0.5)
                    .lineTo(new Vector2d(45.00, -36.00))
                    /*.waitSeconds(7)
                    .splineToLinearHeading(new Pose2d(-30.00, -37.00, Math.toRadians(45.00)), Math.toRadians(45))
                    .splineToLinearHeading(new Pose2d(-40.00, -50.00, Math.toRadians(90.00)), Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(-38.49, -14.23, Math.toRadians(0.00)), Math.toRadians(0))
                    //.waitSeconds(10)
                    .splineToLinearHeading(new Pose2d(36.04, -24.19, Math.toRadians(315.)), Math.toRadians(315))
                    .splineToLinearHeading(new Pose2d(40.00, -43.5, Math.toRadians(0.00)), Math.toRadians(0.00))
                    .waitSeconds(0.2)
                    .addTemporalMarker( () -> { robotBase.armSubsystem.armDropOffLowestPos();})
                    .addTemporalMarker( () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                    .waitSeconds(1)
                    .splineToConstantHeading(new Vector2d(53,-43.5), Math.toRadians(0))
                    .waitSeconds(0.5)
                    .addTemporalMarker( () -> { robotBase.leftClawSubsystem.clawOpen();})
                    .waitSeconds(0.2)
                    .splineToConstantHeading(new Vector2d(43,-43.5), Math.toRadians(0))
                    .waitSeconds(0.2)
                    .addTemporalMarker( () -> { robotBase.leftWristSubsystem.wristPickup();})
                    .addTemporalMarker( () -> { robotBase.armSubsystem.armGrabbingPosition();})*/
                    .build();

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);

        OuterPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, -36.00, Math.toRadians(360)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(45,-61), Math.toRadians(360.0))
                .splineToConstantHeading(new Vector2d(59,-61), Math.toRadians(360.0))
                .build();

        InnerPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, -36.00, Math.toRadians(360)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(45,-10), Math.toRadians(360.0))
                .splineToConstantHeading(new Vector2d(61,-10), Math.toRadians(360.0))
                .build();

            robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);
            parkLocation = InnerPark;
            robotBase.parkSide = RobotBase.ParkSide.INNER;
    }
    @Override
    public void init_loop(){

        autoChassisController.readButtons();
        if (autoChassisController.wasJustPressed((GamepadKeys.Button.Y))) {
            if (robotBase.parkSide == RobotBase.ParkSide.INNER) {
                robotBase.parkSide = RobotBase.ParkSide.OUTER;
                parkLocation = OuterPark;
            } else {
                robotBase.parkSide = RobotBase.ParkSide.INNER;
                parkLocation = InnerPark;
            }
        }
        robotBase.propPosition = robotBase.huskyLensSubsystem.getLocation(robotBase.alliance, robotBase.startPosition);
        telemetry.addData("InitLoop","true");
        telemetry.addData("Detection",(robotBase.propPosition));
        telemetry.addData("Park Side", (robotBase.parkSide));
        telemetry.update();

    }
    @Override
    public void start(){

        if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(MiddleSpike);
        } else if (robotBase.propPosition == RobotBase.PropPosition.LEFT) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(LeftSpike);
        } else {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(RightSpike);
        }
        currentRouteState = RedLeft.CurrentRouteState.TRAJECTORY_1;

    }
    @Override
    public void loop(){
        switch (currentRouteState) {
            case TRAJECTORY_1:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = RedLeft.CurrentRouteState.PARKING;
                    robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(parkLocation);
                }
        }
        robotBase.mecanumDriveSubsystem.update();
        CommandScheduler.getInstance().run();
    }
    @Override
    public void stop(){
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
        DataStorageSubsystem.alliance = robotBase.alliance.RED;
    }
}
