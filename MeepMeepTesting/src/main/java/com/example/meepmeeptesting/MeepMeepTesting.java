package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //Code goes in here
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14.4, -61.5, Math.toRadians(180)))
                .strafeToConstantHeading(new Vector2d(15,-31.5))

                .strafeToConstantHeading(new Vector2d(15,-34))
                .splineToLinearHeading(new Pose2d(27,-40,Math.toRadians(270)),Math.toRadians(360))
                .splineToLinearHeading(new Pose2d(46,-10,Math.toRadians(270)),Math.toRadians(360))
                .strafeToLinearHeading(new Vector2d(46,-57),Math.toRadians(270))

                        .waitSeconds(0.01)
                .strafeToConstantHeading(new Vector2d(46,-10))
                .splineToConstantHeading(new Vector2d(58, -10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58, -57), Math.toRadians(270.00))


                .build()
        );



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}