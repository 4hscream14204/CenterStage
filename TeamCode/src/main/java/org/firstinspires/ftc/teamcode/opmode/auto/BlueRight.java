package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;

@Autonomous(name = "BlueRight")
public class BlueRight extends OpMode {
    public RobotBase robotBase;


    public TrajectorySequence LeftSpike;
    public TrajectorySequence MiddleSpike;
    public TrajectorySequence RightSpike;

    private TrajectorySequence InnerPark;
    private TrajectorySequence OuterPark;
    private TrajectorySequence parkLocation;

    public Pose2d startPose;

    public GamepadEx autoChassisController;

    @Override
    public void init(){
        autoChassisController = new GamepadEx(gamepad1);
        robotBase = new RobotBase(hardwareMap);
        robotBase.parkSide = RobotBase.ParkSide.INNER;
        robotBase.alliance = RobotBase.Alliance.BLUE;
        robotBase.startPosition = RobotBase.StartPosition.RIGHT;
        robotBase.leftClawSubsystem.clawClose();
        robotBase.leftWristSubsystem.wristEscape();
        startPose = new Pose2d(-38.35, 63.3, Math.toRadians(270.00));

        LeftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-41, 63.3, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-28.00, 39.00, Math.toRadians(315.00)), Math.toRadians(315.00))
                .lineToSplineHeading(new Pose2d(-40.00, 50.00, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-20.00, 12.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                //.waitSeconds(10)
                .splineTo(new Vector2d(36.04, 24.19), Math.toRadians(45.))
                .splineToSplineHeading(new Pose2d(45.00, 43.00, Math.toRadians(0.00)), Math.toRadians(90.00))
                .waitSeconds(2.5)
                .addTemporalMarker(7.5, () -> { robotBase.armSubsystem.armDropOffPos();})
                .addTemporalMarker(8, () -> { robotBase.leftWristSubsystem.wristDropOff();})
                .lineTo(new Vector2d(55,42))
                .waitSeconds(1)
                .lineTo(new Vector2d(40, 28))
                .addTemporalMarker(10.5, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .addTemporalMarker(11.5, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .addTemporalMarker(12, () -> { robotBase.armSubsystem.armGrabbingPosition();})
                .build();


        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-41, 63.3, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-36.00, 34.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .lineToLinearHeading(new Pose2d(-34, 43, Math.toRadians(315.00)))
                .splineToSplineHeading(new Pose2d(-5.00, 38.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(45.00, 36.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(1.5)
                .addTemporalMarker(6.5, () -> { robotBase.armSubsystem.armDropOffPos();})
                .addTemporalMarker(7, () -> { robotBase.leftWristSubsystem.wristDropOff();})
                .lineTo(new Vector2d(52,36))
                .waitSeconds(2.5)
                .lineTo(new Vector2d(40, 28))
                .addTemporalMarker(8.5, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .addTemporalMarker(9.5, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .addTemporalMarker(10, () -> { robotBase.armSubsystem.armGrabbingPosition();})
                .build();

        RightSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-41, 63.3, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-45.00, 38.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .lineToConstantHeading(new Vector2d(-35.00, 50.00))
                .splineToLinearHeading(new Pose2d(-25.00, 12.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineToLinearHeading(new Pose2d(25.00, 12.00, Math.toRadians(0.00)))
                .splineToLinearHeading(new Pose2d(45.00, 29.50, Math.toRadians(0.00)), Math.toRadians(0.00))
              /*  .waitSeconds(1.5)
                .addTemporalMarker(6.5, () -> { robotBase.armSubsystem.armDropOffPos();})
                .addTemporalMarker(7, () -> { robotBase.leftWristSubsystem.wristDropOff();})
                .lineTo(new Vector2d(52,36))
                .waitSeconds(2.5)
                .lineTo(new Vector2d(40, 28))
                .addTemporalMarker(8.5, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .addTemporalMarker(9.5, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .addTemporalMarker(10, () -> { robotBase.armSubsystem.armGrabbingPosition();}) */
                .build();

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);

        OuterPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45, 36, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(55.00, 61.00), Math.toRadians(0.00))
                .build();

        InnerPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45, 36, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(55.00, 12.00), Math.toRadians(0.00))
                .build();
        parkLocation = InnerPark;
    }
    @Override
    public void init_loop() {
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
        public void start () {
            if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
                robotBase.mecanumDriveSubsystem.followTrajectorySequence(MiddleSpike);
                //robotBase.grabber.drop();
                //robotBase.mecanumDrive.followTrajectorySequence(RedRightCenterInner2);
            } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
                robotBase.mecanumDriveSubsystem.followTrajectorySequence(RightSpike);
            } else {
                robotBase.mecanumDriveSubsystem.followTrajectorySequence(LeftSpike);
            }
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(parkLocation);

        }
        @Override
        public void loop () {

        }
        @Override
        public void stop () {
            Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double dblCurrentHeading = angles.firstAngle;
            DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
            DataStorageSubsystem.alliance = robotBase.alliance.BLUE;
        }
    }
