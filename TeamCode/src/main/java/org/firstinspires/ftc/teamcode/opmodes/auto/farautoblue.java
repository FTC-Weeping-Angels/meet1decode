package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.intake;
import org.firstinspires.ftc.teamcode.hardware.shooter;
import org.firstinspires.ftc.teamcode.hardware.turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "farblue", group = "auto")
public class farautoblue extends OpMode {
    private Follower follower;
    private org.firstinspires.ftc.teamcode.hardware.intake intake;
    private org.firstinspires.ftc.teamcode.hardware.shooter shooter;
    private org.firstinspires.ftc.teamcode.hardware.turret turret;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime waitTimer1 = new ElapsedTime();
    private ElapsedTime waitTimer2 = new ElapsedTime();
    private ElapsedTime waitTimer3 = new ElapsedTime();
    private int pathState;
    //private Path scorePreload;
    private PathChain scorePreload, intakeset1, scoreset1, intakeset2,scoreset2, parkpath ;
    private final Pose startPose = new Pose(64, 10, Math.toRadians(90));
    public Pose intake1mid = new Pose(64, 42, Math.toRadians(0));
    private final Pose endintake = new Pose(14, 36, Math.toRadians(180));
    private final Pose score1shootpose = new Pose(56, 19, Math.toRadians(90));
    // private final Pose preloadshootpose = new Pose(83, 90, Math.toRadians(0));


    public Pose parkpose = new Pose(57, 35, Math.toRadians(90));

    private final double MAX_POWER = 1; // Maximum speed (0.0 to 1.0)
    private final double STOP_THRESHOLD = 100;
    private final double shooterspeed =.86;// Stop movement if within this many ticks of the target

    //    // Define Target Ticks (Assuming 360 degree rotation takes 1000 ticks for example)
//    private final int FORWARD_TICKS = 0;
//    private final int LEFT_TICKS = 250;
//    private final int RIGHT_TICKS = -250;
    private final double P_GAIN = 0.00025;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        // scorePreload = new Path(new BezierLine(startPose, preloadshootpose));
        //  scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preloadshootpose.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        startPose.mirror();
        intake1mid.mirror();
        endintake.mirror();
        score1shootpose.mirror();
        parkpose.mirror();

        intakeset1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, intake1mid, endintake))
                .setTangentHeadingInterpolation()
                .build();
        scoreset1 = follower.pathBuilder()
                .addPath(new BezierLine(endintake, score1shootpose))
                .setLinearHeadingInterpolation(endintake.getHeading(), score1shootpose.getHeading())
                .setTimeoutConstraint(300)
                .setBrakingStrength(2)
                .build();
        parkpath = follower.pathBuilder()
                // follower.setMaxPower(.25);
                .addPath(new BezierLine(score1shootpose, parkpose))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(5)
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // First path to depo preload

                shooter.setshooter(shooterspeed);
                turret.moveTurretToPosition(1900);

                waitTimer2.reset();
                setPathState(12);
                break;

            case 12: // First path to depo preload
                if(waitTimer2.seconds() >= 3){
                    intake.setSpinner(-1);
                    shooter.setKickerspeed(0);
                    intake.setTransferspeed(1);
                    waitTimer1.reset();
                    setPathState(100);
                }
                break;

            //case 11:
            case 100:
                if (waitTimer1.seconds() >= 4.5) {
                    shooter.setshooter(0);
                    shooter.setKickerspeed(0.75);
                    //intake.setTransferspeed(0.5);
                    intake.setSpinner(-1);
                    follower.setMaxPower(1);
                    follower.followPath(intakeset1, false);

                    waitTimer2.reset();
                    setPathState(222);
                }
                break;
            case 222:
                if (follower.getCurrentTValue()>=0.3) {

                    follower.setMaxPower(.3);
                    shooter.setshooter(shooterspeed);

                    waitTimer2.reset();
                    setPathState(20);
                }

                break;
            case 20: // First path to depo preload
                if (!follower.isBusy() && waitTimer2.seconds() >= 0.5) {
                    //follower.setMaxPower(.75);

                    follower.setMaxPower(1);
                    follower.followPath(scoreset1, false);
                    follower.setHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.04, 0.005));
                    //  shooter.setshooter(shooterspeed);
                    waitTimer1.reset();
                    setPathState(21);
                }

                break;
//
            case 21: // First path to depo preload
                if (!follower.isBusy() && waitTimer1.seconds() >= 0.5) {
                    //follower.setMaxPower(.75);
                    intake.setTransferspeed(1);
                    shooter.setKickerspeed(0);
                    waitTimer2.reset();
                    setPathState(30);//////////////////////////////
                }

                break;

            case 30:
                if (!follower.isBusy() && waitTimer2.seconds() >= 5.5) {
                    shooter.setshooter(0);
                    //intake.setTransferspeed(0.5);

                    waitTimer1.reset();
                    setPathState(99);
                }
                break;
            case 99: // First path to depo preload
                if (!follower.isBusy() && waitTimer1.seconds() >= 1) {
                    //follower.setMaxPower(.75);
                    follower.followPath(parkpath, false);
                    shooter.setshooter(0);
                    shooter.setKickerspeed(0.5);
                    intake.setTransferspeed(0.5);
                    intake.setSpinner(0);
                    waitTimer3.reset();
                    setPathState(999);
                }

                break;
            case 999: // First path to depo preload
                if (!follower.isBusy() && waitTimer3.seconds() >= 1) {
                    //follower.setMaxPower(.75);

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
        telemetry.addData("turretposticks:", turret.getturretposticks());
        telemetry.addData("headingerror:", follower.getHeadingError());
        telemetry.addData("heading:", follower.getHeading());

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

        this.turret = new turret() ;

        turret.init(hardwareMap);

        shooter.setPitchPosition(0.83);
        turret.resetencoder();
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
