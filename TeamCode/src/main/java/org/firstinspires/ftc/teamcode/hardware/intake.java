package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

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
        TRANSFEROFF,// just starts the spinner, then moves on

    }

    // ---- Mechanisms ---- //
    private Servo transfer;

    private Servo blocker;
    private Servo transferled;
    private DcMotorEx spinner;
    private DigitalChannel breakbeam;
    DistanceSensor thirdball;
    ElapsedTime transferTime = new ElapsedTime();
    // ---- Sensors ---- //
    public double gatedown = 0.62;
    public double gateup = 0.37;

    private boolean started = false;




    private IntakeState intakeState;
    private TransferState transferState;

    public void init(HardwareMap hwMap) {


        // this.wrist = hwMap.get(ServoImplEx.class, "intake_wrist");

        this.spinner = hwMap.get(DcMotorEx.class, "intake");
        this.transfer = hwMap.get(ServoImplEx.class, "transfer");
        this.blocker = hwMap.get(ServoImplEx.class, "blocker");
        this.transferled = hwMap.get(ServoImplEx.class, "ledtransfer");

        this.thirdball = hwMap.get(DistanceSensor.class, "thirdball");
        this.breakbeam = hwMap.get(DigitalChannel.class, "breakbeam");

        this.spinner.setDirection(DcMotorEx.Direction.REVERSE);

        this.breakbeam.setMode(DigitalChannel.Mode.INPUT);

        this.intakeState = IntakeState.IDLE;
        this.transferState = TransferState.IDLE;

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

    public void settransferledposition(double position) {
        transferled.setPosition(position);
    }
    public void led3ballsdetected() {
        transferled.setPosition(0.5);
    }
    public void noballsdetected() {
        transferled.setPosition(0.29);
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
    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }
    public void setTransferState(TransferState transferState) {this.transferState = transferState;}

    public boolean breakBeamBoolean() {

            return breakbeam.getState();

    }

    public boolean ballthreeinrobot(){
       if (thirdball.getDistance(DistanceUnit.CM)<=6){
        return true;

       }
       return false;
    }
    public boolean emptyrobot(){
        if (thirdball.getDistance(DistanceUnit.CM)<=8){
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

    //    public Limelight getLimelight() {
//        return limelight;
//    }
    public String getIntakeAMPS(){
        return "Intake Slides Amps: " + String.format("%.2f", spinner.getCurrent(CurrentUnit.AMPS)) + "\n" +
                "Spinner Amps: " + String.format("%.2f", spinner.getCurrent(CurrentUnit.AMPS));
    }
}

