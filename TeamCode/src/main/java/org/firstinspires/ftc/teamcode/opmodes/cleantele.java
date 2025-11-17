package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.intake.IntakeState;
import org.firstinspires.ftc.teamcode.hardware.intake.TransferState;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.hardware.drive;

import org.firstinspires.ftc.teamcode.hardware.turret;
import org.firstinspires.ftc.teamcode.hardware.intake;
import org.firstinspires.ftc.teamcode.hardware.shooter;
import org.firstinspires.ftc.teamcode.hardware.shooter.ShooterState;



@TeleOp(name="smtele", group="Linear OpMode")
public class cleantele extends LinearOpMode {

    // -- Hardware -- //
    private intake intake;
    private drive drivetrain;
    private shooter shooter;
    private turret turret;
    private final double MAX_POWER = 1; // Maximum speed (0.0 to 1.0)
    private final double STOP_THRESHOLD = 100; // Stop movement if within this many ticks of the target

//    // Define Target Ticks (Assuming 360 degree rotation takes 1000 ticks for example)
//    private final int FORWARD_TICKS = 0;
//    private final int LEFT_TICKS = 250;
//    private final int RIGHT_TICKS = -250;
    private final double P_GAIN = 0.00025;

    private final double speed = 1.0;
    ElapsedTime transferTime2 = new ElapsedTime();

    private final double farpos= 0.83;
    private final double closepos = 0.68;
    private final double sidepos = 0.63;





    @Override
    public void runOpMode() {

        initDrive();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        Gamepad previousGamepad2 = new Gamepad();

        turret.resetencoder();


        waitForStart();
        shooter.setPitchPosition(0.58);

        while (opModeIsActive() && !isStopRequested()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            drivetrain.Speed = gamepad1.left_stick_y;  // Inverted forward/backward movement
            drivetrain.Strafe = gamepad1.left_stick_x;
            drivetrain.Turn = -gamepad1.right_stick_x * 0.7;  // Turning speed cut by 50%

            drivetrain.leftFrontPower = Range.clip(drivetrain.Speed - drivetrain.Strafe + drivetrain.Turn, -1, 1);
            drivetrain.rightFrontPower = Range.clip(drivetrain.Speed + drivetrain.Strafe - drivetrain.Turn, -1, 1);
            drivetrain.leftRearPower = Range.clip(drivetrain.Speed + drivetrain.Strafe + drivetrain.Turn, -1, 1);
            drivetrain.rightRearPower = Range.clip(drivetrain.Speed - drivetrain.Strafe - drivetrain.Turn, -1, 1);

            drivetrain.updateMotorPowers();
            //    ------------- GAMEPAD 2 CONTROLS ----------- //
            if (currentGamepad1.right_trigger > 0.5 && previousGamepad1.right_trigger > 0.5) {
                intake.setIntakeState(IntakeState.INTAKING);
                intake.setTransferState(TransferState.INTAKING);


            } else if (currentGamepad1.left_trigger > 0.5 && previousGamepad1.left_trigger > 0.5) {  // Change to gamepad2 if that's what you need.
                intake.setIntakeState(IntakeState.BACKSPIN);
                intake.setTransferState(TransferState.REVERSE);
            }  else if(currentGamepad1.right_bumper ){
                intake.setIntakeState(IntakeState.INTAKING);
            } else if (currentGamepad1.square) {
                shooter.setShooterState(ShooterState.BACKSPIN);
                intake.setTransferState(TransferState.REVERSE);
                //

            } else{
                intake.setIntakeState(IntakeState.STOP_SPINNER);
                intake.setTransferState(TransferState.TRANSFEROFF);
            }
            if (currentGamepad1.squareWasReleased()) {
                shooter.setShooterState(ShooterState.STOP_SHOOTER);
            }

            if(intake.ballthreeinrobot()){
                intake.led3ballsdetected();
            } else if(!intake.ballthreeinrobot()){
                intake.noballsdetected();
            }

            if (currentGamepad1.triangle && !previousGamepad1.triangle) {
                 shooter.setShooterState(ShooterState.FIREFAR);
            }

            if (currentGamepad1.circle && !previousGamepad1.circle) {
                shooter.setShooterState(ShooterState.FIREREDUCED);
            }





            if (currentGamepad1.cross && !previousGamepad1.cross) {  // Change to gamepad2 if that's what you need.
                shooter.setShooterState(ShooterState.STOP_SHOOTER);
                intake.setTransferState(TransferState.TRANSFEROFF);

            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {  // Change to gamepad2 if that's what you need.
                intake.setTransferState(TransferState.FIRE);
               // intake.setIntakeState(IntakeState.INTAKING);

            } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {  // Change to gamepad2 if that's what you need.
                intake.setTransferState(TransferState.REVERSE);
            } else if(currentGamepad1.leftBumperWasReleased() || currentGamepad1.rightBumperWasReleased()) {
                intake.setTransferState(TransferState.TRANSFEROFF);
                //intake.setIntakeState(IntakeState.STOP_SPINNER);

            }

           /// ???????
            if (currentGamepad2.right_bumper) {  // Change to gamepad2 if that's what you need.
               turret.setturretspeed(0.4);

            } else if (currentGamepad2.left_bumper) {  // Change to gamepad2 if that's what you need.
                turret.setturretspeed(0.6);
            }  else if (currentGamepad2.right_trigger > 0.5 && previousGamepad2.right_trigger > 0.5) {  // Change to gamepad2 if that's what you need.
                turret.setturretspeed(0.2);

            } else if (currentGamepad2.left_trigger > 0.5 && previousGamepad2.left_trigger > 0.5) {  // Change to gamepad2 if that's what you need.
                turret.setturretspeed(0.8);
            } else {
                turret.setturretspeed(0.5);
            }


            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {  // Change to gamepad2 if that's what you need.
                turret.setturretspeed(0.2);

            } else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {  // Change to gamepad2 if that's what you need.
                turret.setturretspeed(0.8);
            } else if(currentGamepad1.dpadLeftWasReleased() || currentGamepad1.dpadRightWasReleased()) {
                turret.setturretspeed(0.5);
            }
            ////?????
//            if (currentGamepad2.a && !previousGamepad2.a) {  // Change to gamepad2 if that's what you need.
//                moveTurretToPosition(3000);
//
//            }
//
//            if (currentGamepad2.y && !previousGamepad2.y) {  // Change to gamepad2 if that's what you need.
//                moveTurretToPosition(-2500);
//
//            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {  // Change to gamepad2 if that's what you need.
                shooter.setPitchPosition(shooter.getPitchPosition()+0.05);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {  // Change to gamepad2 if that's what you need.
                shooter.setPitchPosition(shooter.getPitchPosition()-0.05);
            }



            if (currentGamepad2.triangle && !previousGamepad2.triangle) {
                shooter.setPitchPosition(closepos);
            }

            if (currentGamepad2.square && !previousGamepad2.square) {
                shooter.setPitchPosition(sidepos);
                //

            }
            if (currentGamepad2.circle && !previousGamepad2.circle) {
                shooter.setPitchPosition(sidepos);
                //

            }
            if (currentGamepad2.cross && !previousGamepad2.cross) {
                shooter.setPitchPosition(farpos);
                //

            }
//

            switch (intake.getIntakeState()) {
                case IDLE:
                    break;
                case INTAKING:
                    intake.setSpinner(-1);
                    intake.setIntakeState(IntakeState.IDLE);
                    break;
                case STOP_SPINNER:
                    intake.setSpinner(0);
                    intake.setIntakeState(IntakeState.IDLE);
                    break;
                case BACKSPIN:
                    intake.setSpinner(1);
                    intake.setIntakeState(IntakeState.IDLE);
                    break;
            }

            switch (intake.getTransferState()) {
                case IDLE:
                    break;
                case INTAKING:
                    intake.setTransferspeed(1);
                    shooter.setKickerspeed(0.63);
                    intake.setTransferState(TransferState.SCANNING);
                    break;
                case SCANNING:
                    if(!intake.breakBeamBoolean()){
                        intake.setTransferspeed(0.5);
                        shooter.setKickerspeed(0.5);
                        intake.setTransferState(TransferState.TRANSFEROFF);
                    }
                    //intake.setIntakeState(IntakeState.SCANNING);
                    break;
                case TRANSFEROFF:
                    intake.setTransferspeed(0.5);
                    shooter.setKickerspeed(0.5);
                    intake.setTransferState(TransferState.IDLE);
                    break;
                case FIRE:
                    intake.setTransferspeed(1);
                    shooter.setKickerspeed(0);
                    intake.setTransferState(TransferState.IDLE);
                    break;
                case REVERSE:
                    intake.setTransferspeed(0);
                    shooter.setKickerspeed(1);
                    intake.setTransferState(TransferState.IDLE);
                    break;
            }

            switch (shooter.getshooterState()) {
                case IDLE:
                    break;
                case FIREFULLPOWER:
                   shooter.setshooter(0.95);
                    shooter.setShooterState(ShooterState.IDLE);
                    break;
                case FIREREDUCED:
                    shooter.setshooter(0.78);
                    shooter.setShooterState(ShooterState.IDLE);
                    break;
                case FIREFAR:
                    shooter.setshooter(0.87);
                    shooter.setShooterState(ShooterState.IDLE);
                    break;
                case BACKSPIN:
                    shooter.setshooter(-1);
                    shooter.setShooterState(ShooterState.IDLE);
                    break;
                case STOP_SHOOTER:
                    shooter.setshooter(0);
                    shooter.setShooterState(ShooterState.IDLE);
                    break;

            }




            updateTelemetry();
        }

    }

    public void moveTurretToPosition(int targetTicks) {

        double currentPosition;
        double error;
        double calculatedPower;
        double servoCommand;

        while ( Math.abs(targetTicks - turret.getturretposticks()) > STOP_THRESHOLD) {

            currentPosition = turret.getturretposticks();

            // 1. Calculate Error (in Ticks)
            error = targetTicks - currentPosition;

            // 2. Calculate Power (P Control)
            calculatedPower = error * P_GAIN;

            // 3. Limit and Clip Power
            calculatedPower = Range.clip(calculatedPower, -MAX_POWER, MAX_POWER);

            // 4. Convert Power (-1.0 to 1.0) to Servo Command (0.0 to 1.0)
            servoCommand = 0.5 + (calculatedPower / 2.0);

            // 5. Command the Servo
           turret.setturretspeed(servoCommand);

            // --- Telemetry for Debugging ---
//        telemetry.addData("Target", "%.1f°", targetAngleDegrees);
//        telemetry.addData("Current", "%.1f°", turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE);
//        telemetry.addData("Error Ticks", (int) error);
//        telemetry.addData("Power", String.format("%.3f", calculatedPower));
//        telemetry.update();
        }

        // Stop the Servo
        turret.setturretspeed(0.5);
//        telemetry.addData("Movement Complete", "Stopped at %.1f°", turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE);
//        telemetry.update();
      //  return;
    }


    /**
     * Sends debug data to control hub
     */
    private void updateTelemetry () {
       // telemetry.addData("Intake Data", 1);
        telemetry.addData("pitch_pos:", shooter.getPitchPosition());
        telemetry.addData("kickerspeed:", shooter.getKickerPosition());
        telemetry.addData("shooter:", shooter.getLrhino().getPower());
        telemetry.addData("DISTANCESENSOR", intake.laserdistance());
        telemetry.addData("breakbeam detected:", intake.breakBeamBoolean());
        telemetry.addData("intakestate:", intake.getIntakeState());
        telemetry.addData("shooterstate:", shooter.getshooterState());
        telemetry.addData("transferstate:", intake.getTransferState());

        telemetry.addData("turretposticks:", turret.getturretposticks());
        telemetry.addData("turretposdeg:", turret.getturretpos());
      //  telemetry.addData("axonpos:", turret.axonencoderpos());
        // telemetry.addData("Spinner AMPS: ", intake.getSpinner().getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }


    /**
     * Call this once to initialize hardware when TeleOp initializes.
     */
    private void initDrive () {
//        this.controllers = new Controllers(this);

        this.intake = new intake();
        intake.init(hardwareMap);


        this.drivetrain = new drive();
        drivetrain.init(hardwareMap);

        this.shooter = new shooter();
        shooter.init(hardwareMap);////

        this.turret = new turret();
        turret.init(hardwareMap);

    }

    public drive getdrivetrain () {
        return drivetrain;
    }


    public intake getIntake () {
        return intake;
    }

    public shooter getShooter () {
        return shooter;
    }
    public turret getTurret () {
        return turret;
    }


}
