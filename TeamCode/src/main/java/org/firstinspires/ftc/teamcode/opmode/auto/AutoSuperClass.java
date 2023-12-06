package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;

public class AutoSuperClass extends OpMode {


    RobotBase robotBase = new RobotBase(hardwareMap);

    int blockTopCoordinate;
    int blockLeftCoordinate;

    //enum for which spike tape the team prop is on
    enum PropLocation {
        LEFT,
        MIDDLE,
        RIGHT,
    }

    //enum for if you are on the left or right starting position
    enum Sides {
        LEFT,
        RIGHT,
    }

    //enum for where to park at the end of autonomous
    enum ParkSide {
        INNER,
        OUTER,
    }



    //Making first instance of proplocation
    PropLocation propLocation;

    //making first instance of sides
    Sides side;

    ParkSide parkSide;

    public TrajectorySequence redLsLp;
    public TrajectorySequence redLsMp;
    public TrajectorySequence redLsRp;

    public TrajectorySequence redRsLp;

    public TrajectorySequence redRsMp;

    public TrajectorySequence redRsRp;

    public TrajectorySequence blueLsLp;

    public TrajectorySequence blueLsMp;

    public TrajectorySequence blueLsRp;

    public TrajectorySequence blueRsLp;
    public TrajectorySequence blueRsMp;
    public TrajectorySequence blueRsRp;

    public TrajectorySequence redBackLeft;

    public TrajectorySequence redBackMiddle;

    public TrajectorySequence redBackRight;

    public TrajectorySequence blueBackLeft;

    public TrajectorySequence blueBackMiddle;

    public TrajectorySequence blueBackRight;

    public TrajectorySequence redParkOuter;

    public TrajectorySequence redParkInner;

    public TrajectorySequence blueParkOuter;

    public TrajectorySequence blueParkInner;

    public GamepadEx chassisController;

    public GamepadEx ChassisControllerRightB;

    @Override
    public void init() {

        chassisController = new GamepadEx(gamepad1);

        // setting default position of proplocation
        propLocation = PropLocation.LEFT;


        // setting default starting side
        side = Sides.LEFT;

        // setting parking position
        parkSide = ParkSide.OUTER;

        // START OF TRAJECTORIES

        Pose2d startRedLsLp = new Pose2d(-38.35, -63.3, Math.toRadians(90));

        robotBase.mecanumDrive.setPoseEstimate(startRedLsLp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence redLsLp = robotBase.mecanumDrive.trajectorySequenceBuilder(startRedLsLp)
                .splineToSplineHeading(new Pose2d(-43, -36.00, Math.toRadians(135.00)), Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-36.00, -36.00, Math.toRadians(90.00)), Math.toRadians(180.00))
                .waitSeconds(15)
                .splineToSplineHeading(new Pose2d(-31, -56, Math. toRadians(-39)), Math .toRadians(-39))
                .splineToSplineHeading(new Pose2d(-12, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(45, 36, Math.toRadians(180)), Math.toRadians(90))
                .build();


        Pose2d startRedLsMp = new Pose2d(-39, -63, Math.toRadians(270));

        robotBase.mecanumDrive.setPoseEstimate(startRedLsMp);

//Red alliance on the left starting side and team prop on middle spike tape position
        TrajectorySequence redLsMp = robotBase.mecanumDrive.trajectorySequenceBuilder(startRedLsMp)
                .splineToSplineHeading(new Pose2d(-36, -29.00, Math.toRadians(135.00)), Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-36.00, -36.00, Math.toRadians(90.00)), Math.toRadians(180.00))
                .waitSeconds(15)
                .splineToSplineHeading(new Pose2d(-31, -56, Math. toRadians(-39)), Math .toRadians(-39))
                .splineToSplineHeading(new Pose2d(-12, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(45, 36, Math.toRadians(180)), Math.toRadians(90))
                .build();


        Pose2d startRedLsRp = new Pose2d(-39, -63, Math.toRadians(270));

        robotBase.mecanumDrive.setPoseEstimate(startRedLsRp);

//Red alliance on the left starting side and team prop on right spike tape position
        TrajectorySequence redLsRp = robotBase.mecanumDrive.trajectorySequenceBuilder(startRedLsRp)
                .splineToSplineHeading(new Pose2d(-36, -29.00, Math.toRadians(135.00)), Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-36.00, -36.00, Math.toRadians(90.00)), Math.toRadians(180.00))
                .waitSeconds(15)
                .splineToSplineHeading(new Pose2d(-31, -56, Math. toRadians(-39)), Math .toRadians(-39))
                .splineToSplineHeading(new Pose2d(-12, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(45, -36, Math.toRadians(180)), Math.toRadians(90))
                .build();


        Pose2d startRedRsLp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDrive.setPoseEstimate(startRedRsLp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence redRsLp = robotBase.mecanumDrive.trajectorySequenceBuilder(startRedRsLp)
                .splineToSplineHeading(new Pose2d(6.00, -36.00, Math.toRadians(135.00)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(18, -49, Math.toRadians(0.00 )), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(45, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();


        Pose2d startRedRsMp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDrive.setPoseEstimate(startRedRsMp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence redRsMp = robotBase.mecanumDrive.trajectorySequenceBuilder(startRedRsMp)
                .splineToSplineHeading(new Pose2d(12.00, -34.00, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(18, -49, Math.toRadians(0.00 )), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(45, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();


        Pose2d startRedRsRp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDrive.setPoseEstimate(startRedRsRp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence redRsRp = robotBase.mecanumDrive.trajectorySequenceBuilder(startRedRsRp)
                .splineToSplineHeading(new Pose2d(20.00, -37.00, Math.toRadians(60.00)), Math.toRadians(60.00))
                .splineToSplineHeading(new Pose2d(18, -49, Math.toRadians(0.00 )), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(45, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();


        Pose2d startBlueLsLp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDrive.setPoseEstimate(startBlueLsLp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence blueLsLp = robotBase.mecanumDrive.trajectorySequenceBuilder(startBlueLsLp)
                .splineToSplineHeading(new Pose2d(20.00, 37.00, Math.toRadians(300.00)), Math.toRadians(300.00))
                .splineToSplineHeading(new Pose2d(18, 49, Math.toRadians(0.00 )), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(36.0, 36.0, Math.toRadians(180)), Math.toRadians(270))
                .build();

        Pose2d startBlueLsMp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDrive.setPoseEstimate(startBlueLsMp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence blueLsMp = robotBase.mecanumDrive.trajectorySequenceBuilder(startBlueLsMp)
                .splineToSplineHeading(new Pose2d(12.00, 34.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(18, 49, Math.toRadians(0.00 )), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(36.0, 36.0, Math.toRadians(180)), Math.toRadians(270))
                .build();

        Pose2d startBlueLsRp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDrive.setPoseEstimate(startBlueLsRp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence blueLsRp = robotBase.mecanumDrive.trajectorySequenceBuilder(startBlueLsRp)
                .splineToSplineHeading(new Pose2d(6.00, 36.00, Math.toRadians(225.00)), Math.toRadians(225))
                .splineToSplineHeading(new Pose2d(18, 49, Math.toRadians(0.00 )), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(36.0, 36.0, Math.toRadians(180)), Math.toRadians(270))
                .build();

        Pose2d startBlueRsLp = new Pose2d(-38.35, 63.3, Math.toRadians(270.00));

        robotBase.mecanumDrive.setPoseEstimate(startBlueRsLp);

//Blue Alliance on the right side and the left spike position
        TrajectorySequence blueRsLp = robotBase.mecanumDrive.trajectorySequenceBuilder(startBlueRsLp)
                .splineToSplineHeading(new Pose2d(-30.00, 36.00, Math.toRadians(315.00)), Math.toRadians(315.00))
                .splineToSplineHeading(new Pose2d(-29, 37, Math.toRadians(300.00)), Math.toRadians(300.00))
                .splineToSplineHeading(new Pose2d(-34, 38, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(15)
                .splineToSplineHeading(new Pose2d(-31, 59, Math.toRadians(40)), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-11, 60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13.5, 55.5, Math.toRadians(44.5)), Math.toRadians(44.5))
                .splineToSplineHeading(new Pose2d(36.0, 36.0, Math.toRadians(180)), Math.toRadians(270))
                .build();

        Pose2d startBlueRsMp = new Pose2d(-38.35, 63.3, Math.toRadians(270.00));

        robotBase.mecanumDrive.setPoseEstimate(startBlueRsLp);

//Blue Alliance on the right side and the middle spike position
        TrajectorySequence blueRsMp = robotBase.mecanumDrive.trajectorySequenceBuilder(startBlueRsMp)
                .splineToSplineHeading(new Pose2d(-36.00, 30.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(-29, 37, Math.toRadians(300.00)), Math.toRadians(300.00))
                .splineToSplineHeading(new Pose2d(-34, 38, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(15)
                .splineToSplineHeading(new Pose2d(-31, 59, Math.toRadians(40)), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-11, 60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13.5, 55.5, Math.toRadians(44.5)), Math.toRadians(44.5))
                .splineToSplineHeading(new Pose2d(36.0, 36.0, Math.toRadians(180)), Math.toRadians(270))
                .build();

        Pose2d startBlueRsRp = new Pose2d(-38.35, 63.3, Math.toRadians(270.00));

        robotBase.mecanumDrive.setPoseEstimate(startBlueRsLp);

//Blue Alliance on the right side and the right spike position
        TrajectorySequence blueRsRp = robotBase.mecanumDrive.trajectorySequenceBuilder(startBlueRsRp)
                .splineToSplineHeading(new Pose2d(-41.00, 36.00, Math.toRadians(225)), Math.toRadians(225.00))
                .splineToSplineHeading(new Pose2d(-29, 37, Math.toRadians(300.00)), Math.toRadians(300.00))
                .splineToSplineHeading(new Pose2d(-34, 38, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(15)
                .splineToSplineHeading(new Pose2d(-31, 59, Math.toRadians(40)), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-11, 60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13.5, 55.5, Math.toRadians(44.5)), Math.toRadians(44.5))
                .splineToSplineHeading(new Pose2d(36.0, 36.0, Math.toRadians(180)), Math.toRadians(270))
                .build();

        //Drop off position on the left april tag of the back board
        TrajectorySequence redBackLeft = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(45, -36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(54, -29, Math.toRadians(180 )), Math.toRadians(0.00))
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .splineToSplineHeading(new Pose2d(48, -36.00, Math.toRadians(180 )), Math.toRadians(180))
                .build();

        //Drop off position on the middle april tag of the back board
        TrajectorySequence redBackMiddle = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(45, -36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(50.00, -37.00, Math.toRadians(180 )), Math.toRadians(0.00))
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .splineToSplineHeading(new Pose2d(48, -36.00, Math.toRadians(180 )), Math.toRadians(180))
                .build();

        //Drop off position on the right april tag of the back board
        TrajectorySequence redBackRight = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(45, -36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(50.00, -43.00, Math.toRadians(180 )), Math.toRadians(0.00))
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .splineToSplineHeading(new Pose2d(48, -36.00, Math.toRadians(180 )), Math.toRadians(180))
                .build();

        //Drop off position on the left april tag of the back board BLUE
        TrajectorySequence blueBackLeft = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(50.00, 43.00, Math.toRadians(180 )), Math.toRadians(0.00))
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .splineToSplineHeading(new Pose2d(48, 36.00, Math.toRadians(180 )), Math.toRadians(180))
                .build();

        //Drop off position on the middle april tag of the back board BLUE
        TrajectorySequence blueBackMiddle = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(50.00, 37.00, Math.toRadians(180 )), Math.toRadians(0.00))
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .splineToSplineHeading(new Pose2d(48, 36.00, Math.toRadians(180 )), Math.toRadians(180))
                .build();

        //Drop off position on the right april tag of the back board BLUE
        TrajectorySequence blueBackRight = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(50.00, 29.00, Math.toRadians(180 )), Math.toRadians(0.00))
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .splineToSplineHeading(new Pose2d(48, 36.00, Math.toRadians(180 )), Math.toRadians(180))
                .build();


        TrajectorySequence blueParkOuter = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(48, 36, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(62, 57.00, Math.toRadians(180 )), Math.toRadians(180))

                .build();

        TrajectorySequence blueParkInner = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(48, 36, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(63, 14.00, Math.toRadians(180 )), Math.toRadians(180))

                .build();

        TrajectorySequence redParkOuter = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(48, -36, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(62, -57.00, Math.toRadians(180 )), Math.toRadians(180))

                .build();

        TrajectorySequence redParkInner = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(48, -36, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(63, -14.00, Math.toRadians(180 )), Math.toRadians(180))

                .build();
    }

    @Override
    public void init_loop() {


    HuskyLens.Block[] blocks = robotBase.huskyLens.blocks();

    for (int i = 0; i < blocks.length; i++){
        //Assign the block attributes to variables to use later
        blockTopCoordinate = blocks[i].top;
        blockLeftCoordinate = blocks[i].left;

    if (blockLeftCoordinate < 100){
        propLocation = PropLocation.LEFT;
    }else if (blockLeftCoordinate > 200) {
        propLocation = PropLocation.RIGHT;
    } else {
        propLocation = PropLocation.MIDDLE;
    }
    }

        if (chassisController.wasJustPressed(GamepadKeys.Button.B)) {
            robotBase.startPosition = RobotBase.StartPosition.LEFT;
            telemetry.addData(">>", "Left");
        }

        if (chassisController.wasJustPressed(GamepadKeys.Button.Y)) {
            robotBase.startPosition = RobotBase.StartPosition.RIGHT;
            telemetry.addData(">>", "Left");
        }



//What trajectory to run if on Red alliance left side and prop on left tape
        if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.LEFT){
             robotBase.mecanumDrive.followTrajectorySequence(redLsLp);
            robotBase.mecanumDrive.followTrajectorySequence(redBackLeft);

        }
//What trajectory to run if on Red alliance left side and prop on middle tape
        if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.MIDDLE){
            robotBase.mecanumDrive.followTrajectorySequence(redLsMp);
            robotBase.mecanumDrive.followTrajectorySequence(redBackMiddle);
        }
//What trajectory to run if on Red alliance left side and prop on right tape
        if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.RIGHT){
            robotBase.mecanumDrive.followTrajectorySequence(redLsRp);
            robotBase.mecanumDrive.followTrajectorySequence(redBackRight);
        }
//What trajectory to run if on Red alliance right side and prop on left tape
        if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.RIGHT && propLocation == PropLocation.LEFT){
              robotBase.mecanumDrive.followTrajectorySequence(redRsLp);
            robotBase.mecanumDrive.followTrajectorySequence(redBackLeft);
        }
//What trajectory to run if on Red alliance right side and prop on middle tape
        if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.RIGHT && propLocation == PropLocation.MIDDLE){
            robotBase.mecanumDrive.followTrajectorySequence(redRsMp);
            robotBase.mecanumDrive.followTrajectorySequence(redBackMiddle);
        }
//What trajectory to run if on Red alliance right side and prop on right tape
        if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.RIGHT && propLocation == PropLocation.RIGHT){
            robotBase.mecanumDrive.followTrajectorySequence(redRsRp);
            robotBase.mecanumDrive.followTrajectorySequence(redBackRight);
        }
//What trajectory to run if on Blue alliance left side and prop on left tape
        if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.LEFT && propLocation == PropLocation.LEFT){
            robotBase.mecanumDrive.followTrajectorySequence(blueLsLp);
            robotBase.mecanumDrive.followTrajectorySequence(blueBackLeft);
        }
//What trajectory to run if on Blue alliance left side and prop on middle tape
        if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.LEFT && propLocation == PropLocation.MIDDLE){
            robotBase.mecanumDrive.followTrajectorySequence(blueLsMp);
            robotBase.mecanumDrive.followTrajectorySequence(blueBackMiddle);
        }
//What trajectory to run if on Blue alliance left side and prop on right tape
        if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.LEFT && propLocation == PropLocation.RIGHT){
            robotBase.mecanumDrive.followTrajectorySequence(blueLsRp);
            robotBase.mecanumDrive.followTrajectorySequence(blueBackRight);
        }
//What trajectory to run if on Blue alliance right side and prop on left tape
        if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.RIGHT && propLocation == PropLocation.LEFT){
            robotBase.mecanumDrive.followTrajectorySequence(blueLsLp);
            robotBase.mecanumDrive.followTrajectorySequence(blueBackLeft);
        }
//What trajectory to run if on Blue alliance right side and prop on middle tape
        if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.RIGHT && propLocation == PropLocation.MIDDLE){
            robotBase.mecanumDrive.followTrajectorySequence(blueLsMp);
            robotBase.mecanumDrive.followTrajectorySequence(blueBackMiddle);
        }
//What trajectory to run if on Blue alliance right side and prop on right tape
        if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.RIGHT && propLocation == PropLocation.RIGHT){
            robotBase.mecanumDrive.followTrajectorySequence(blueLsRp);
            robotBase.mecanumDrive.followTrajectorySequence(blueBackMiddle);
        }

    }
    @Override
    public void start() {
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(){
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
    }

}
