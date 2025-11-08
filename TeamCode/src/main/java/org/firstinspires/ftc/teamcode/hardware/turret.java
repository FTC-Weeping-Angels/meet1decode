package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class turret {


    // ---- Mechanisms ---- //
    private ServoImplEx leftturret;
    private ServoImplEx rightturret;


    // ---- Sensors ---- //

    public void init(HardwareMap hwMap) {


        this.leftturret = hwMap.get(ServoImplEx.class, "lturret");
        this.rightturret = hwMap.get(ServoImplEx.class, "rturret");
//

    }


    public void setturretspeed(double power) {
       leftturret.setPosition(power);
       rightturret.setPosition(-power);
    }


    public double getlturretPosition() {
        return leftturret.getPosition();
    }

    public Servo getlturret() {
        return leftturret;
    }

    public String getData() {

        StringBuilder data = new StringBuilder();


        return data.toString();
    }
}