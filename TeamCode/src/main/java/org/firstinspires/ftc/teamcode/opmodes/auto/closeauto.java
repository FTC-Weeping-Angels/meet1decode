package org.firstinspires.ftc.teamcode.opmodes.auto; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.hardware.drive;
import org.firstinspires.ftc.teamcode.hardware.turret;
import org.firstinspires.ftc.teamcode.hardware.intake;
import org.firstinspires.ftc.teamcode.hardware.shooter;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "close", group = "auto")
public class closeauto extends OpMode {
    private Follower follower;
    private intake intake;
    private shooter shooter;
    private turret turret;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime waitTimer1 = new ElapsedTime();
    private int pathState;
    //private Path scorePreload;
    private PathChain scorePreload;
    private final Pose startPose = new Pose(120, 124, Math.toRadians(35));
    private final Pose preloadshootpose = new Pose(83, 88, Math.toRadians(0));

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
       // scorePreload = new Path(new BezierLine(startPose, preloadshootpose));
      //  scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preloadshootpose.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preloadshootpose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadshootpose.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // First path to depo preload
                turret.setturretspeed(1);
                follower.setMaxPower(.75);

                follower.followPath(scorePreload, false);
                waitTimer1.reset();
                setPathState(111);
                break;
            case 111: // First path to depo preload
                if(waitTimer1.milliseconds() >= 570){
                    turret.setturretspeed(0.5);
                    setPathState(11);
                }


                break;
            case 11: // First path to depo preload
                if(follower.getCurrentTValue() >= 0.6){
                    shooter.setshooter(0.87);
                    setPathState(12);
                }


                break;
            case 12: // First path to depo preload
               if(!follower.isBusy()){
                intake.firefortime(3000);
                   waitTimer1.reset();
                   setPathState(100);
               }


                break;

            //case 11:
            case 100:
                if (!follower.isBusy() && waitTimer1.seconds() >= 5) {
                   // shooter.setshooter(0);
                    setPathState(-1);
                }
                break;

        }
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        this.intake = new intake();
        intake.init(hardwareMap);

        this.shooter = new shooter();
        shooter.init(hardwareMap);////

        this.turret = new turret();
        turret.init(hardwareMap);

        shooter.setPitchPosition(0.81);
       // turret.turretautospin(1,250);
    }
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }





    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(10);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


}