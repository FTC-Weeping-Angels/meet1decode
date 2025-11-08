package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

/**
 * This is an class which, with proper tuning of a Limelight camera, will detect an AprilTag,
 * specifically 20 or 24, which are the ones on the blue and red goals respectively.
 * There are two components: the tracking itself, and the oscillations while it does not detect
 * the AprilTags, moving the camera back and forth until it finds the AprilTag, upon whose
 * detection it homes in on. Commented out are the additions of mecanum drive, which can be
 * uncommented or swapped out for different drive or autonomous movement.
 *
 * @author Tomas Roth - 11139 Knight Owls
 * @version 1.0, 9/18/2025
 */



@Autonomous(name="turrettesting")
public class turretvisiontesting extends OpMode {

    private Limelight3A limelight;
    private Servo turret1;
    private Servo turret2;

    private double targetx;
    private IMU imu;
    private int turretOscillationDirection;
    // public DcMotor RightFront, RightBack, LeftFront, LeftBack; // drive motors

    double regularDivBy = 1;

    public DcMotor initMotor(String name, boolean direction)
    {
        //direction true=forward false=reverse
        DcMotor temp = hardwareMap.get(DcMotor.class, name);
        if (!direction) temp.setDirection(DcMotorSimple.Direction.REVERSE);
        temp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return temp;
    }

    @Override
    public void init() {
        turret1 = hardwareMap.servo.get("turret1");
        turret2 = hardwareMap.servo.get("turret2");
        turretOscillationDirection = 0; // 0=left, 1=right

        // // drive motors
        // RightFront = initMotor("rightFront", true);
        // LeftFront = initMotor("leftFront", true);
        // RightBack = initMotor("rightRear", true);
        // LeftBack = initMotor("leftRear", false);

        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {


        // Get orientation and Tx, Ty, Ta
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        }

        // Turret code - detects AprilTag and goes to its location
        if (llResult != null) {
            targetx = llResult.getTx();
            telemetry.addData("targetx", llResult.getTx());
            if ((targetx <= -3.5) && (turret1.getPosition() > 0.0025)) {
                turret1.setPosition(turret1.getPosition() - 0.0020);
            } else if ((targetx >= 3.5) && (turret1.getPosition() < 0.9975)) {
                turret1.setPosition(turret1.getPosition() + 0.0020);
            }
        }

        //Oscillation when not detecting AprilTag
        if ((llResult.getTx() == 0.0) && (llResult.getTy() == 0.0)) {
            telemetry.addData("Detection", "False");
            if ((turretOscillationDirection == 1) && (turret1.getPosition() < 0.996)) { //if its going right and less than 0.999
                turret1.setPosition(turret1.getPosition() + 0.0035);
                if (turret1.getPosition() >= 0.996) {
                    turret1.setPosition(0.994);
                    turretOscillationDirection = 0; // now go left
                }
            } else if ((turretOscillationDirection == 0) && (turret1.getPosition() > 0.0040)) { //if its going left and more than 0.001
                turret1.setPosition(turret1.getPosition() - 0.0035);
                if (turret1.getPosition() <= 0.0040) {
                    turret1.setPosition(0.0045);
                    turretOscillationDirection = 1; // now go right
                }
            } else {
                telemetry.addData("ERROR!!", "NOT IN OSCILLATION RANGE");
                turret1.setPosition(0.5);
            }
        }

        telemetry.addData("Xseen", llResult.getTx());
        telemetry.addData("turret1Position", turret1.getPosition());
        telemetry.addData("turret2Position", turret2.getPosition());
        telemetry.addData("turretOscillationDirection", turretOscillationDirection);

        telemetry.update();
    }
}