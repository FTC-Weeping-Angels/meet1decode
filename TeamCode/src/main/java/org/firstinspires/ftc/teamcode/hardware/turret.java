package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;


public class turret {



    private ElapsedTime waitTimer2 = new ElapsedTime();
    // ---- Mechanisms ---- //
    private ServoImplEx leftturret;
    private ServoImplEx rightturret;
    private DcMotorEx turretencoder;
    private double currentpos;
    private final double MAX_POWER = 1; // Maximum speed (0.0 to 1.0)
    private final double STOP_THRESHOLD = 100; // Stop movement if within this many ticks of the target

    //    // Define Target Ticks (Assuming 360 degree rotation takes 1000 ticks for example)
//    private final int FORWARD_TICKS = 0;
//    private final int LEFT_TICKS = 250;
//    private final int RIGHT_TICKS = -250;
    private final double P_GAIN = 0.00025;



    //get our analog input from the hardwareMap
    //  private AnalogInput axonencoder;

    // get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
    // double axonpos;

    // ---- Sensors ---- //

    public void init(HardwareMap hwMap) {

        //  this.axonencoder = hwMap.get(AnalogInput.class, "axonencoder");
        this.leftturret = hwMap.get(ServoImplEx.class, "lturret");
        this.rightturret = hwMap.get(ServoImplEx.class, "rturret");
        this.turretencoder = hwMap.get(DcMotorEx.class, "turretencoder");
        turretencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
   // public void runOpMode() {}

    public void turretautospin(double power, double time) {
        waitTimer2.reset();
        setturretspeed(power);

        if (waitTimer2.milliseconds() >= time) {
            setturretspeed(0.5);
        }
    }

    public void setturretspeed(double power) {
        leftturret.setPosition(power);
        rightturret.setPosition(power);
    }

//    public double axonencoderpos() {
//        double axonpos = axonencoder.getVoltage() / 3.3 * 360;
//
//       return axonpos;
//    }

//
    public void resetencoder() {
        turretencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
//    public double getlservoturretPos() {
//        return leftturret.getPosition();
//    }

    public double getturretpos() {

        return turretencoder.getCurrentPosition() * 0.01050813;
    }

    public double getturretposticks() {

        return turretencoder.getCurrentPosition();
    }

    public void moveTurretToPosition(int targetTicks) {

        double currentPosition;
        double error;
        double calculatedPower;
        double servoCommand;

        while ( Math.abs(targetTicks - getturretposticks()) > STOP_THRESHOLD) {

            currentPosition = getturretposticks();

            // 1. Calculate Error (in Ticks)
            error = targetTicks - currentPosition;

            // 2. Calculate Power (P Control)
            calculatedPower = error * P_GAIN;

            // 3. Limit and Clip Power
            calculatedPower = Range.clip(calculatedPower, -MAX_POWER, MAX_POWER);

            // 4. Convert Power (-1.0 to 1.0) to Servo Command (0.0 to 1.0)
            servoCommand = 0.5 + (calculatedPower / 2.0);

            // 5. Command the Servo
            setturretspeed(servoCommand);

            // --- Telemetry for Debugging ---
//        telemetry.addData("Target", "%.1f°", targetAngleDegrees);
//        telemetry.addData("Current", "%.1f°", turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE);
//        telemetry.addData("Error Ticks", (int) error);
//        telemetry.addData("Power", String.format("%.3f", calculatedPower));
//        telemetry.update();
        }

        // Stop the Servo
        setturretspeed(0.5);
//        telemetry.addData("Movement Complete", "Stopped at %.1f°", turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE);
//        telemetry.update();
        //  return;
    }
    public Servo getlturret() {
        return leftturret;
    }

    public String getData() {

        StringBuilder data = new StringBuilder();


        return data.toString();
    }


}

