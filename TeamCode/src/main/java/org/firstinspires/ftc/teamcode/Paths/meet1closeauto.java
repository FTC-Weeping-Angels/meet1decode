package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;

public class meet1closeauto{
    private final Follower follower;
    public Pose start = new Pose(21, 122, Math.toRadians(90));
    public Pose scoreEnd = new Pose(48, 96, Math.toRadians(135));

    private int index = 0;
    //TODO: ACTUALLY ADD ALL PATHS
    private static final int PATH_COUNT = 1;

    public meet1closeauto(Robot rob){
        this.follower = rob.follower;

        if(rob.alliances.equals(Alliance.RED)) {
            start = start.mirror();
            scoreEnd = scoreEnd.mirror();
        }
    }

    public PathChain score() {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                start,
                                new Pose(56,94),
                                scoreEnd
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), scoreEnd.getHeading())
                .setBrakingStrength(5)
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return score();
            default: return null;
        }

    }

    public boolean hasNext() { return index < PATH_COUNT; }

    public void reset() {index = 0;}
}
