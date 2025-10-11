package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d; // NOTE: no ".geometry" in 1.0
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30).turn(Math.toRadians(90)).lineToY(30)
                .turn(Math.toRadians(90)).lineToX(100   ).turn(Math.toRadians(90))
                .lineToY(0).turn(Math.toRadians(90)).build());

        mm.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true).setBackgroundAlpha(0.95f)
                .addEntity(bot).start();
    }
}
