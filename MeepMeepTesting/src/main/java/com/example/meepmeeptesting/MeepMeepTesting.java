package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import sun.font.TrueTypeFont;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12.5,17.75)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(17.5, -63, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(3.00, -36.00, Math.toRadians(135.00)), Math.toRadians(135.00))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(15,-40, Math.toRadians(0.00)),Math.toRadians(0.00))
                                .splineToSplineHeading(new Pose2d(40.00, -41.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .waitSeconds(0.2)
                                .lineTo(new Vector2d(50.00,-41.00))
                                .waitSeconds(0.5)
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(40.00, -36.00))
                                .waitSeconds(0.2)
                                .build());
        //Don't delete lines below
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}