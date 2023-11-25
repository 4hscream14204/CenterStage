package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

    //Making first instance of proplocation
    PropLocation propLocation;

    //making first instance of sides
    Sides side;

    @Override
    public void init() {

        // setting default position of proplocation
        propLocation = PropLocation.LEFT;


        // setting default starting side
        side = Sides.LEFT;

        // START OF TRAJECTORIES


        Pose2d startRedLsLp = new Pose2d(-38.35, -63.3, Math.toRadians(90));

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startRedLsLp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence redLsLp = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startRedLsLp)



                .build();


        Pose2d startRedLsMp = new Pose2d(-39, -63, Math.toRadians(270));

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startRedLsMp);

//Red alliance on the left starting side and team prop on middle spike tape position
        TrajectorySequence redLsMp = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startRedLsMp)

                .build();


        Pose2d startRedLsRp = new Pose2d(-39, -63, Math.toRadians(270));

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startRedLsRp);

//Red alliance on the left starting side and team prop on middle spike tape position
        TrajectorySequence redLsRp = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startRedLsRp)

                .build();


        Pose2d startRedRsLp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startRedRsLp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence redRsLp = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startRedRsLp)
                .splineToSplineHeading(new Pose2d(6.00, -36.00, Math.toRadians(135.00)), Math.toRadians(180))
                .build();


        Pose2d startRedRsMp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startRedRsMp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence redRsMp = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startRedRsMp)
                .splineToSplineHeading(new Pose2d(12.00, -34.00, Math.toRadians(90)), Math.toRadians(90))


                .build();


        Pose2d startRedRsRp = new Pose2d(15.00, -63.00, Math.toRadians(90));

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startRedRsRp);

//Red alliance on the left starting side and team prop on left spike tape position
        TrajectorySequence redRsRp = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startRedRsRp)
                .splineToSplineHeading(new Pose2d(20.00, -37.00, Math.toRadians(60.00)), Math.toRadians(60.00))

                .build();


//common spot for robot after spike tape drop off
        TrajectorySequence redRsPrep = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startRedRsLp)
                .splineToSplineHeading(new Pose2d(18, -49, Math.toRadians(0.00 )), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(36, -34, Math.toRadians(0.00)), Math.toRadians(180))
                .build();

        //Drop off position on the left april tag of the back board
        TrajectorySequence redBackLeft = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(redRsPrep.end())
                .splineToSplineHeading(new Pose2d(54, -29, Math.toRadians(0.00 )), Math.toRadians(0.00))
                .lineToLinearHeading(new Pose2d(52, -29, Math.toRadians(0.00)))
                .build();
    }

@Override
    public void init_loop() {

        /*
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
         */
//What trajectory to run if on Red alliance left side and prop on left tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.LEFT){
       // robotBase.MecanumDrive.followTrajectorySequence();

    }
//What trajectory to run if on Red alliance left side and prop on middle tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.MIDDLE){

    }
//What trajectory to run if on Red alliance left side and prop on right tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.RIGHT){

    }
//What trajectory to run if on Red alliance right side and prop on left tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.RIGHT && propLocation == PropLocation.LEFT){
      //  robotBase.MecanumDrive.followTrajectorySequence(redRsLp);
    }
//What trajectory to run if on Red alliance right side and prop on middle tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.RIGHT && propLocation == PropLocation.MIDDLE){

    }
//What trajectory to run if on Red alliance right side and prop on right tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.RIGHT && propLocation == PropLocation.RIGHT){

    }
//What trajectory to run if on Blue alliance left side and prop on left tape
    if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.LEFT && propLocation == PropLocation.LEFT){

    }
//What trajectory to run if on Blue alliance left side and prop on middle tape
    if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.LEFT && propLocation == PropLocation.MIDDLE){

    }
//What trajectory to run if on Blue alliance left side and prop on right tape
    if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.LEFT && propLocation == PropLocation.RIGHT){

    }
//What trajectory to run if on Blue alliance right side and prop on left tape
    if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.RIGHT && propLocation == PropLocation.LEFT){

    }
//What trajectory to run if on Blue alliance right side and prop on middle tape
    if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.RIGHT && propLocation == PropLocation.MIDDLE){

    }
//What trajectory to run if on Blue alliance right side and prop on right tape
    if(robotBase.alliance == RobotBase.Alliance.BLUE && side == Sides.RIGHT && propLocation == PropLocation.RIGHT){

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
