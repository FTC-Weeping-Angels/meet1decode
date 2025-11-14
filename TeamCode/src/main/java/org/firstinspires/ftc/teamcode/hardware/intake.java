package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.shooter;

import java.util.HashMap;
import java.util.Map;

public class intake {


    public enum IntakeState {
        IDLE, // Idle
        INTAKING,
        SCANNING,
        TRANSFEROFF,// just starts the spinner, then moves on
        BACKSPIN,
        STOP_SPINNER,


    }

    public enum TransferState {
        IDLE, // Idle
        SCANNING,
        TRANSFEROFF,
        FIRE,
        REVERSE,
        INTAKING// just starts the spinner, then moves on

    }

    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    private shooter shooter;
    // ---- Mechanisms ---- //
    private ElapsedTime waitTimer2 = new ElapsedTime();
    private Servo transfer;

    private Servo blocker;
    private Servo transferled;
    private DcMotorEx spinner;
    private DigitalChannel breakbeam;
    AnalogInput thirdball;
    ElapsedTime transferTime = new ElapsedTime();
    // ---- Sensors ---- //
    public double gatedown = 0.62;
    public double gateup = 0.37;

    private boolean started = false;




    private IntakeState intakeState;
    private TransferState transferState;

    public intake(HardwareMap hardwareMap) {


        // this.wrist = hwMap.get(ServoImplEx.class, "intake_wrist");

        this.spinner = hardwareMap.get(DcMotorEx.class, "intake");
        this.transfer = hardwareMap.get(ServoImplEx.class, "transfer");
        this.blocker = hardwareMap.get(ServoImplEx.class, "blocker");
        this.transferled = hardwareMap.get(ServoImplEx.class, "ledtransfer");

        this.thirdball = hardwareMap.get(AnalogInput.class, "thirdball");
        this.breakbeam = hardwareMap.get(DigitalChannel.class, "breakbeam");

        this.spinner.setDirection(DcMotorEx.Direction.REVERSE);

        this.breakbeam.setMode(DigitalChannel.Mode.INPUT);

        this.intakeState = IntakeState.IDLE;
        this.transferState = TransferState.IDLE;
        //this.shooter = new shooter();
      //  shooter;

    }





    public void setSpinner(double power) {
        spinner.setPower(power);
    }

    /**
     * Check if the intake is back at its original position by using the magnetic sensor
     * @return if the magnetic sensor is active
     */
    public void setTransferspeed(double position) {
        transfer.setPosition(position);
    }
    public void setblockposition(double position) {
        blocker.setPosition(position);
    }


//    public void firefortime( double time) {
//        waitTimer2.reset();
//        shooter.setKickerspeed(0);
//        transfer.setPosition(1);
//        if(waitTimer2.milliseconds() >= time){
//            shooter.setKickerspeed(0.5);
//            transfer.setPosition(0.5);
//        }
//    }

    public void settransferledposition(double position) {
        transferled.setPosition(position);
    }
    public void led3ballsdetected() {
        transferled.setPosition(0.29);
    }
    public void noballsdetected() {
        transferled.setPosition(0.5);
    }


    public void setTransferCyclein(double seconds) {
        transferTime.reset();
        if(transferTime.seconds() >= seconds){
            transfer.setPosition(1);
        } else {
            transfer.setPosition(0.5);
        }
    }
    public void setTransferCycleout(double seconds) {
        transferTime.reset();
        if(transferTime.seconds() >= seconds){
            transfer.setPosition(0);
        } else {
            transfer.setPosition(0.5);
        }
    }

    public void setintakefortime(double seconds) {
        transferTime.reset();
        spinner.setPower(1);
        if(transferTime.seconds() >= seconds) {
            spinner.setPower(0);
        }
    }
    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }
    public void setTransferState(TransferState transferState) {this.transferState = transferState;}

    public boolean breakBeamBoolean() {

            return breakbeam.getState();

    }

public double laserdistance(){
    // Read sensor voltage (0.0–3.3V)
    double volts = thirdball.getVoltage();

    // Convert voltage to distance in millimeters (linear mapping)
    double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
return distanceMM;
}

    public boolean ballthreeinrobot(){
       if (laserdistance()<=60){
        return true;

       }
       return false;
    }
    public boolean emptyrobot(){
        if (laserdistance() >=70){
            return true;

        }
        return false;
    }
    public IntakeState getIntakeState() {
        return intakeState;
    }
    public TransferState getTransferState() {
        return transferState;
    }


    public Servo getTransfer() {
        return transfer;
    }
    public Servo gettransferled() {
        return transferled;
    }
    public Servo getBlocker() {
        return blocker;
    }

    public DcMotorEx getSpinner() {
        return spinner;
    }



    /**
     * Check what type of sample that the intake color sensor detects
     * @return sample type (RED, YELLOW, BLUE)
     */


    public String getData() {

        StringBuilder data = new StringBuilder();






        return data.toString();
    }


    public boolean isStarted() {
        return started;
    }

    public void setStarted(boolean started) {
        this.started = started;
    }

    public double getSpinnerVoltage() {
        return spinner.getCurrent(CurrentUnit.AMPS);
    }
//    public shooter getShooter () {
//        return shooter;
//    }

    //    public Limelight getLimelight() {
//        return limelight;
//    }
    public String getIntakeAMPS(){
        return "Intake Slides Amps: " + String.format("%.2f", spinner.getCurrent(CurrentUnit.AMPS)) + "\n" +
                "Spinner Amps: " + String.format("%.2f", spinner.getCurrent(CurrentUnit.AMPS));
    }
}

