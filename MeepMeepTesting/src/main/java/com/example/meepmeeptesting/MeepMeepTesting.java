package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(650);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14.4, -61.5, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(15,-31.5))
                        .strafeToLinearHeading(new Vector2d(34,-42),Math.toRadians(360))

                        //Be above the block
                        .strafeToConstantHeading(new Vector2d(36,-5))
                        .strafeToConstantHeading(new Vector2d(46,-5))
                .strafeToConstantHeading(new Vector2d(46,-52))
                                .strafeToConstantHeading(new Vector2d(46,-5) )


                              .strafeToConstantHeading(new Vector2d(55,-5))
                              .strafeToConstantHeading(new Vector2d(55,-52))
                .strafeToConstantHeading(new Vector2d(55, -48))
                                .strafeToConstantHeading(new Vector2d(26.5,-59.2))
                                .strafeToLinearHeading(new Vector2d(9, -28),Math.toRadians(180))
.strafeToLinearHeading(new Vector2d(26.5, -54),Math.toRadians(360))
                .strafeTo(new Vector2d(26.5,-58.8))
                                        .strafeToLinearHeading(new Vector2d(6, -28),Math.toRadians(180))
.strafeToLinearHeading(new Vector2d(26.5, -54),Math.toRadians(360))


                .strafeTo(new Vector2d(26.5,-58.8))
                                         .strafeToLinearHeading(new Vector2d(3, -28), Math.toRadians(180))

                        .build()
        );

        mm.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}