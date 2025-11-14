package org.firstinspires.ftc.teamcode.vision;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;

import java.util.List;


public class Limelight {
    private Limelight3A Limelight;
    private Alliance alliances;
    //Shoot being tags 20 and 24, Zone being 21-23
    //TODO: Make Functions for Zone
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;
    public Limelight(HardwareMap hardwareMap, Alliance alliance) {
        alliances = alliance;
        Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        switchToShoot();
    }

    public void start() {
        Limelight.start();
    }

    public void stop() {
        Limelight.stop();
    }

    public void pause() {
        Limelight.pause();
    }

    public double distanceFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = Limelight.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null) {
            double x = (target.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch) + 8; // right/left from tag
            double z = (target.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 8; // forward/back from tag

            Vector e = new Vector();
            e.setOrthogonalComponents(x, z);
            return e.getMagnitude();
        }

        return 0;
    }

    public double distanceFromBlue() {
        return distanceFromTag(20);
    }

    public double distanceFromRed() {
        return distanceFromTag(24);
    }

    public double angleFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = Limelight.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null)
            return target.getTargetXDegrees();

        return 0;
    }

    public double angleFromBlue() {
        return angleFromTag(20);
    }

    public double angleFromRed() {
        return angleFromTag(24);
    }

    public double angleFromShoot() {
        return alliances == Alliance.BLUE ? angleFromBlue() : angleFromRed();
    }

    public double distanceFromShoot() {
        return alliances == Alliance.BLUE ? distanceFromBlue() : distanceFromRed();
    }

    public void switchToShoot() {
        if (pipeline != shoot)
            Limelight.pipelineSwitch(shoot);
        Limelight.setPollRateHz(20);
        Limelight.start();
    }
}

