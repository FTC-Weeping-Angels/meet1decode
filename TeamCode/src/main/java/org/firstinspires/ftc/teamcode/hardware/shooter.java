package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class shooter {


    // ---- Mechanisms ---- //
    private ServoImplEx pitch;
    private DcMotorEx rrhino;
    private DcMotorEx lrhino;
    private Servo kicker;


    // ---- Sensors ---- //

    public void init(HardwareMap hwMap) {

        this.lrhino = hwMap.get(DcMotorEx.class, "lrhino");
        this.rrhino = hwMap.get(DcMotorEx.class, "rrhino");
        this.pitch = hwMap.get(ServoImplEx.class, "pitch");
        this.kicker = hwMap.get(ServoImplEx.class, "kicker");

        this.lrhino.setDirection(DcMotorEx.Direction.FORWARD);
        this.rrhino.setDirection(DcMotorEx.Direction.REVERSE);
//

    }


    public void setshooter(double power) {rrhino.setPower(power); lrhino.setPower(power);}
    public void setKickerspeed(double position) {
        kicker.setPosition(position);
    }
    public void setPitchPosition(double position) {
        pitch.setPosition(position);
    }

    public double getPitchPosition() {
        return pitch.getPosition();
    }
    public double getKickerPosition() {
        return kicker.getPosition();
    }
    public DcMotorEx getRrhino() {return rrhino;}
    public DcMotorEx getLrhino() {return lrhino;}
    public Servo getKicker() {
        return kicker;
    }

    public String getData() {

        StringBuilder data = new StringBuilder();


        return data.toString();
    }
}