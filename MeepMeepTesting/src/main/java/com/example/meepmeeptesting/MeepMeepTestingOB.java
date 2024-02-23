package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingOB {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41, 63.3, Math.toRadians(270)))
                          /*      .splineTo(new Vector2d(-40, 36.11), Math.toRadians(225.00))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-36.00, 60.00), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(45.00, 33.00), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(50.00, 33.00), Math.toRadians(0.00))
                                .lineTo(new Vector2d(40.00, 33.00)) */
                                .splineTo(new Vector2d(-35.82, 33.18), Math.toRadians(270.00))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-36.00, 60.00), Math.toRadians(0.00))
                                //.setReversed(false)
                                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(45.00, 33.00), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(50.00, 33.00), Math.toRadians(0.00))
                                .lineTo(new Vector2d(40.00, 33.00))
                                /*.splineTo(new Vector2d(-30.00, 36.00), Math.toRadians(-37.61))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-36.00, 60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(0.00, 60.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(50.00, 36.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(40.00, 36.00), Math.toRadians(0.00)) */
                                /*.splineTo(new Vector2d(-27.66, 34.16), Math.toRadians(-37.61))
                                .setReversed(true)
                                .splineTo(new Vector2d(-42.25, 51.06), Math.toRadians(110.56))
                                .setReversed(false)
                                .splineTo(new Vector2d(-40.95, 60.16), Math.toRadians(4.16))
                                .setReversed(true)
                                .splineTo(new Vector2d(-25.49, 59.72), Math.toRadians(0.00))
                                .splineTo(new Vector2d(2.53, 58.86), Math.toRadians(-0.55))
                                .splineTo(new Vector2d(50.34, 36.32), Math.toRadians(-1.51))*/
                                .build());
        //Don't delete lines below
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

