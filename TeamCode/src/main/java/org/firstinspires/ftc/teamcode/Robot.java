package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.intake;
import org.firstinspires.ftc.teamcode.hardware.shooter;
import org.firstinspires.ftc.teamcode.hardware.turret;
import org.firstinspires.ftc.teamcode.hardware.drive;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.vision.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class Robot {
//    public final intake intake;
//    public final Limelight limelight;
//    public final shooter shooter;
  //  public final Transfer transfer;
    public final Follower follower;
    public final Alliance alliances;

    private final List<LynxModule> hubs;
    private final Timer loop = new Timer();
    private int loops = 0;
    private double looptime = 0;

    public static Pose endPose = new Pose();

    public Robot(HardwareMap hardwareMap, Alliance alliances){
        this.alliances = alliances;
//        intake = new intake(hardwareMap);
//        shooter = new shooter(hardwareMap);
       // limelight = new Limelight(hardwareMap, alliances);
       // transfer = new Transfer(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : hubs){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        loop.resetTimer();
    }

//    public void periodic(){
//        loops++;
//
//        if (loop.getElapsedTime() % 5 == 0){
//            for(LynxModule hub : hubs) {
//                hub.clearBulkCache();
//            }
//            looptime = (double) loop.getElapsedTime()/loops;
//        }
//        follower.update();
//        shooter.periodic();
//    }
    public void stop(){ endPose = follower.getPose();}

    public double getLoopTime(){return looptime;}
}
