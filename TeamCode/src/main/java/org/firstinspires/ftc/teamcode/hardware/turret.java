package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class turret {

    private ElapsedTime waitTimer2 = new ElapsedTime();
    // ---- Mechanisms ---- //
    private ServoImplEx leftturret;
    private ServoImplEx rightturret;
    private DcMotorEx turretencoder;
    private double currentpos;

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

    public void turretautospin(double power, double time) {
        waitTimer2.reset();
        setturretspeed(power);

        if(waitTimer2.milliseconds() >= time){
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

    public void setturretposition(double targetposition) {

       if(getturretpos() >= targetposition) {
           setturretspeed(0.4);
       }else if (getturretpos() <= targetposition){
               setturretspeed(0.5);
           }




       if(getturretpos() <= targetposition){
           setturretspeed(0.6);
           if (getturretpos() >= targetposition){
               setturretspeed(0.5);
           }
       }


    }

    public void resetencoder() {
        turretencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
//    public double getlservoturretPos() {
//        return leftturret.getPosition();
//    }

    public double getturretpos() {

        return turretencoder.getCurrentPosition() *0.01050813;
    }


    public Servo getlturret() {
        return leftturret;
    }

    public String getData() {

        StringBuilder data = new StringBuilder();


        return data.toString();
    }
}