package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBE {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12.5,17.75)
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41, -63.3, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(-49.00, -46.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                                .lineToConstantHeading(new Vector2d(-38.00, -49.00))
                                .splineToConstantHeading(new Vector2d(-40.00, -35.00), Math.toRadians(90.00))
                                .splineToSplineHeading(new Pose2d(30.00, -12.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .splineToLinearHeading(new Pose2d(38, -25, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .waitSeconds(0.3)
                                .lineTo(new Vector2d(51,-27))
                                .waitSeconds(0.2)
                                //.addDisplacementMarker( () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                                /*.splineToLinearHeading(new Pose2d(38, -27, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .waitSeconds(0.3)
                                .lineTo(new Vector2d(51,-27))
                                .waitSeconds(0.2)
                                //.addDisplacementMarker( () -> { robotBase.leftClawSubsystem.clawOpen();})
                                .waitSeconds(0.2)
                                .lineTo(new Vector2d(40, -36))
                                //.addTemporalMarker( () -> {
                                //    robotBase.leftWristSubsystem.wristPickup();
                                //})
                                .waitSeconds(0.5)
                                //.addTemporalMarker( () -> {
                                //    robotBase.armSubsystem.armGrabbingPosition();
                                //})
                                */
                                .build());
        //Don't delete lines below
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}