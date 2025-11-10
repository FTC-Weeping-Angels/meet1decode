package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooter {

    public enum ShooterState {
        IDLE, // Idle
        // just starts the spinner, then moves on
        BACKSPIN,
        STOP_SHOOTER,
        FIREFULLPOWER,
        FIREREDUCED,

    }
    // ---- Mechanisms ---- //
    private ServoImplEx pitch;
    private DcMotorEx rrhino;
    private DcMotorEx lrhino;
    private Servo kicker;
    private ElapsedTime waitTimer2 = new ElapsedTime();

    private ShooterState shooterState;
    // ---- Sensors ---- //

    public void init(HardwareMap hwMap) {

        this.lrhino = hwMap.get(DcMotorEx.class, "lrhino");
        this.rrhino = hwMap.get(DcMotorEx.class, "rrhino");
        this.pitch = hwMap.get(ServoImplEx.class, "pitch");
        this.kicker = hwMap.get(ServoImplEx.class, "kicker");

        this.lrhino.setDirection(DcMotorEx.Direction.FORWARD);
        this.rrhino.setDirection(DcMotorEx.Direction.REVERSE);

        this.shooterState = shooterState.IDLE;

    }


    public void setshooter(double power) {rrhino.setPower(power); lrhino.setPower(power);}
    public void setKickerspeed(double position) {
        kicker.setPosition(position);
    }
    public void setPitchPosition(double position) {
        pitch.setPosition(position);
    }

    public void shooterfortime(double power, double time) {
        waitTimer2.reset();
        rrhino.setPower(power); lrhino.setPower(power);
        if(waitTimer2.milliseconds() >= time){
            rrhino.setPower(0); lrhino.setPower(0);
        }
    }
    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }
    public double getPitchPosition() {
        return pitch.getPosition();
    }
    public double getKickerPosition() {
        return kicker.getPosition();
    }
    public ShooterState getshooterState() {
        return shooterState;
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