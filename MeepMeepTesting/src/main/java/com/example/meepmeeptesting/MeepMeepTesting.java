package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12.5,17.75)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41, 63.3, Math.toRadians(0.00)))
                                .splineToLinearHeading(new Pose2d(-28.00, 39.00, Math.toRadians(315.00)), Math.toRadians(315.00))
                                .lineToSplineHeading(new Pose2d(-40.00, 50.00, Math.toRadians(270.00)))
                                .splineToSplineHeading(new Pose2d(-52.00, 12.00, Math.toRadians(0.00)), Math.toRadians(270.00))
                                .waitSeconds(2)
                                .splineTo(new Vector2d(27.00, 12.00), Math.toRadians(0))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(43, 40.5),Math.toRadians(0.00))
                                .build());
        //Don't delete lines below
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}