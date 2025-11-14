package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;

public class meet1closeauto{
    private final Follower follower;
    public Pose start = new Pose(120, 124, Math.toRadians(35));
    public Pose preloadshootpose = new Pose(83, 90, Math.toRadians(0));
    public Pose endintake1 = new Pose(118, 86.5, Math.toRadians(-5));
    public Pose intake2mid = new Pose(94, 47, Math.toRadians(0));
    public Pose endintake2 = new Pose(127, 60, Math.toRadians(0));


    private int index = 0;
    //TODO: ACTUALLY ADD ALL PATHS
    private static final int PATH_COUNT = 1;

    public meet1closeauto(Robot rob){
        this.follower = rob.follower;

        if(rob.alliances.equals(Alliance.BLUE)) {
            start = start.mirror();
            preloadshootpose = preloadshootpose.mirror();
            endintake1 = endintake1.mirror();

        }
    }

    public PathChain scorePreload() {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, preloadshootpose))
                .setLinearHeadingInterpolation(start.getHeading(), preloadshootpose.getHeading())
                .setBrakingStrength(5)
                .build();


    }

    public PathChain intakeset1() {
        follower.setMaxPower(.25);
        return follower.pathBuilder()
                .addPath(new BezierLine(preloadshootpose, endintake1))
                .setLinearHeadingInterpolation(preloadshootpose.getHeading(), endintake1.getHeading())
                .setBrakingStrength(5)
                .build();


    }
    public PathChain scoreset1() {
        follower.setMaxPower(1);
        return follower.pathBuilder()
                .addPath(new BezierLine(endintake1, preloadshootpose))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(5)
                .build();


    }
    public PathChain intakeset2() {
        follower.setMaxPower(.25);
        return follower.pathBuilder()
                .addPath(new BezierCurve(preloadshootpose, intake2mid, endintake2))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(5)
                .build();


    }
    public PathChain scoreset2() {
        follower.setMaxPower(1);
        return follower.pathBuilder()
                .addPath(new BezierLine(endintake2, preloadshootpose))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(5)
                .build();


    }

    public PathChain next() {
        switch (index++) {
            case 0: return scorePreload();
            case 1: return intakeset1();
            case 2: return scoreset1();
            case 3: return intakeset2();
            case 4: return scoreset2();
            default: return null;
        }

    }

    public boolean hasNext() { return index < PATH_COUNT; }

    public void reset() {index = 0;}
}
