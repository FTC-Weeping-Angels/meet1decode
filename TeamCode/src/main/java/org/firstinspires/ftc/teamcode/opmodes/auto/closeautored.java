package org.firstinspires.ftc.teamcode.opmodes.auto; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.hardware.turret;
import org.firstinspires.ftc.teamcode.hardware.intake;
import org.firstinspires.ftc.teamcode.hardware.shooter;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "closered", group = "auto")
public class closeautored extends OpMode {
    private Follower follower;
    private intake intake;
    private shooter shooter;
    private turret turret;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime waitTimer1 = new ElapsedTime();
    private ElapsedTime waitTimer2 = new ElapsedTime();
    private int pathState;
    //private Path scorePreload;
    private PathChain scorePreload, intakeset1, scoreset1;
    private final Pose startPose = new Pose(120, 124, Math.toRadians(35));
    private final Pose preloadshootpose = new Pose(83, 90, Math.toRadians(0));
   // private final Pose preloadshootpose = new Pose(83, 90, Math.toRadians(0));
    private final Pose endintake = new Pose(118, 86.5, Math.toRadians(-5));
    private final double MAX_POWER = 1; // Maximum speed (0.0 to 1.0)
    private final double STOP_THRESHOLD = 100;
    private final double shooterspeed =0.72;// Stop movement if within this many ticks of the target

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
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preloadshootpose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadshootpose.getHeading())
                .build();
        intakeset1 = follower.pathBuilder()
                .addPath(new BezierLine(preloadshootpose, endintake))
                .setLinearHeadingInterpolation(preloadshootpose.getHeading(), endintake.getHeading())
                .build();
        scoreset1 = follower.pathBuilder()
                .addPath(new BezierLine(endintake, preloadshootpose))
                .setConstantHeadingInterpolation(0)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // First path to depo preload

                follower.setMaxPower(.95);

                follower.followPath(scorePreload, false);

                waitTimer1.reset();
                setPathState(11);
                break;

            case 11: // First path to depo preload
                if(follower.getCurrentTValue() >= 0.6){
                    moveTurretToPosition(4200);
                    shooter.setshooter(shooterspeed);
                    waitTimer2.reset();
                    setPathState(12);
                }


                break;
            case 12: // First path to depo preload
               if(!follower.isBusy() && waitTimer2.seconds() >= 2){
                   shooter.setKickerspeed(0);
                   intake.setTransferspeed(1);
                   waitTimer1.reset();
                   setPathState(100);
               }


                break;

            //case 11:
            case 100:
                if (!follower.isBusy() && waitTimer1.seconds() >= 4) {
                    shooter.setshooter(0);
                    shooter.setKickerspeed(0.5);
                    //intake.setTransferspeed(0.5);
                    intake.setSpinner(-1);
                    follower.setMaxPower(.25);
                    follower.followPath(intakeset1, false);
                    waitTimer2.reset();
                    setPathState(20);
                }
                break;

            case 20: // First path to depo preload
                if (!follower.isBusy() && waitTimer2.seconds() >= 2) {
                    //follower.setMaxPower(.75);
                    follower.setMaxPower(.95);
                    follower.followPath(scoreset1, false);
                    shooter.setshooter(shooterspeed);
                    waitTimer1.reset();
                    setPathState(21);
                }

                break;

            case 21: // First path to depo preload
                if (!follower.isBusy() && waitTimer1.seconds() >= 2) {
                    //follower.setMaxPower(.75);

                    shooter.setKickerspeed(0);
                    waitTimer1.reset();
                    setPathState(99);
                }

                break;

            case 99: // First path to depo preload
                if (!follower.isBusy() && waitTimer1.seconds() >= 6) {
                    //follower.setMaxPower(.75);
                    shooter.setshooter(0);
                    shooter.setKickerspeed(0.5);
                    intake.setTransferspeed(0.5);
                    intake.setSpinner(0);
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

        shooter.setPitchPosition(0.68);
        turret.resetencoder();
       // turret.turretautospin(1,250);
    }
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }



    public void moveTurretToPosition(int targetTicks) {

        double currentPosition;
        double error;
        double calculatedPower;
        double servoCommand;

        while ( Math.abs(targetTicks - turret.getturretposticks()) > STOP_THRESHOLD) {

            currentPosition = turret.getturretposticks();

            // 1. Calculate Error (in Ticks)
            error = targetTicks - currentPosition;

            // 2. Calculate Power (P Control)
            calculatedPower = error * P_GAIN;

            // 3. Limit and Clip Power
            calculatedPower = Range.clip(calculatedPower, -MAX_POWER, MAX_POWER);

            // 4. Convert Power (-1.0 to 1.0) to Servo Command (0.0 to 1.0)
            servoCommand = 0.5 + (calculatedPower / 2.0);

            // 5. Command the Servo
            turret.setturretspeed(servoCommand);

            // --- Telemetry for Debugging ---
//        telemetry.addData("Target", "%.1f°", targetAngleDegrees);
//        telemetry.addData("Current", "%.1f°", turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE);
//        telemetry.addData("Error Ticks", (int) error);
//        telemetry.addData("Power", String.format("%.3f", calculatedPower));
//        telemetry.update();
        }

        // Stop the Servo
        turret.setturretspeed(0.5);
//        telemetry.addData("Movement Complete", "Stopped at %.1f°", turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE);
//        telemetry.update();
        //  return;
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