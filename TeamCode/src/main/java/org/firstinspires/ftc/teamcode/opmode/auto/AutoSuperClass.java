package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class AutoSuperClass extends OpMode {


    RobotBase robotBase = new RobotBase(hardwareMap);

    int blockTopCoordinate;
    int blockLeftCoordinate;

    enum PropLocation {
        LEFT,
        MIDDLE,
        RIGHT,
    }

    enum Sides {
        LEFT,
        RIGHT,
    }

    PropLocation propLocation;

    Sides side;

    @Override
    public void init() {

        propLocation = PropLocation.LEFT;

        side = Sides.LEFT;

        //RedLeft
        Pose2d redLeftStart = new Pose2d(-40.29, -63.05, Math.toRadians(90));

        robotBase.MecanumDrive.setPoseEstimate(redLeftStart);


        TrajectorySequence redLeft1 = robotBase.MecanumDrive.trajectorySequenceBuilder(redLeftStart)

                .splineToSplineHeading(new Pose2d(-35.89, -34.42, Math.toRadians(90)), Math.toRadians(90))

                .build();

        //RedRight
        Pose2d redRightStart = new Pose2d(-39, -63, Math.toRadians(270));

        robotBase.MecanumDrive.setPoseEstimate(redRightStart);


        TrajectorySequence redRight1 = robotBase.MecanumDrive.trajectorySequenceBuilder(redRightStart)

                .build();

        //BlueLeft
        Pose2d blueLeftStart = new Pose2d(-39, -63, Math.toRadians(270));

        robotBase.MecanumDrive.setPoseEstimate(blueLeftStart);


        TrajectorySequence blueLeft1 = robotBase.MecanumDrive.trajectorySequenceBuilder(blueLeftStart)

                .build();

        //BlueRight
        Pose2d blueRightStart = new Pose2d(-39, -63, Math.toRadians(270));

        robotBase.MecanumDrive.setPoseEstimate(blueRightStart);


        TrajectorySequence blueRight1 = robotBase.MecanumDrive.trajectorySequenceBuilder(blueRightStart)

                .build();
//Trajectory for if the Prop is located on the middle spike tape on the redside
        TrajectorySequence middleRed = robotBase.MecanumDrive.trajectorySequenceBuilder(redLeft1.end())

                .build();
//Trajectory for if the Prop is located on left or right spike tape on the red side
        TrajectorySequence sidesRed = robotBase.MecanumDrive.trajectorySequenceBuilder(redLeft1.end())

                .build();

        TrajectorySequence middleBlue = robotBase.MecanumDrive.trajectorySequenceBuilder(redLeft1.end())

                .build();

        TrajectorySequence sidesBlue = robotBase.MecanumDrive.trajectorySequenceBuilder(redLeft1.end())

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
//What trajectory to run if on Red alliance left side and prop on left tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.LEFT){

    }
//What trajectory to run if on Red alliance left side and prop on middle tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.MIDDLE){

    }
//What trajectory to run if on Red alliance left side and prop on right tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.LEFT && propLocation == PropLocation.RIGHT){

    }
//What trajectory to run if on Red alliance right side and prop on left tape
    if(robotBase.alliance == RobotBase.Alliance.RED && side == Sides.RIGHT && propLocation == PropLocation.LEFT){

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
}
