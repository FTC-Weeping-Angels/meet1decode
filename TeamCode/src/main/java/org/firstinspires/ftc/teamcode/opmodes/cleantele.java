package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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



@TeleOp(name="smtele", group="Linear OpMode")
public class cleantele extends LinearOpMode {


    // -- Hardware -- //
    private intake intake;
    private drive drivetrain;
    private shooter shooter;
    private turret turret;

    private final double speed = 1.0;
    ElapsedTime transferTime2 = new ElapsedTime();





    @Override
    public void runOpMode() {

        initDrive();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        shooter.setPitchPosition(0.5);


        waitForStart();


//        intake.moveExtendo(0, 1);


        while (opModeIsActive() && !isStopRequested()) {

//            controllers.updateCopies(gamepad1, gamepad2);
//            controllers.readInputs();
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // -- GAMEPAD 1 CONTROLS!!! --
            //start intake


            drivetrain.Speed = gamepad1.left_stick_y;  // Inverted forward/backward movement
            drivetrain.Strafe = gamepad1.left_stick_x;
            drivetrain.Turn = -gamepad1.right_stick_x * 0.4;  // Turning speed cut by 50%

            drivetrain.leftFrontPower = Range.clip(drivetrain.Speed - drivetrain.Strafe + drivetrain.Turn, -1, 1);
            drivetrain.rightFrontPower = Range.clip(drivetrain.Speed + drivetrain.Strafe - drivetrain.Turn, -1, 1);
            drivetrain.leftRearPower = Range.clip(drivetrain.Speed + drivetrain.Strafe + drivetrain.Turn, -1, 1);
            drivetrain.rightRearPower = Range.clip(drivetrain.Speed - drivetrain.Strafe - drivetrain.Turn, -1, 1);

            drivetrain.updateMotorPowers();
            //    ------------- GAMEPAD 2 CONTROLS ----------- //
            if (currentGamepad1.right_trigger > 0.5 && previousGamepad1.right_trigger > 0.5) {
                intake.setIntakeState(IntakeState.INTAKING);
                intake.setTransferState(TransferState.SCANNING);


            } else if (currentGamepad1.left_trigger > 0.5 && previousGamepad1.left_trigger > 0.5) {  // Change to gamepad2 if that's what you need.
                intake.setSpinner(1);
                intake.setTransferspeed(1);
            }  else {
                intake.setIntakeState(IntakeState.STOP_SPINNER);
                // intake.setTransferspeed(0.5);
            }
            if(intake.ballthreeinrobot()){
                intake.led3ballsdetected();
            } else if(!intake.ballthreeinrobot()){ intake.noballsdetected();}

            if (currentGamepad1.triangle && !previousGamepad1.triangle) {
                // Change to gamepad2 if that's what you need.
                shooter.setshooter(0.92);
                //transferTime2.reset();
                // if(transferTime2.milliseconds() >= 1500){
                //intake.setTransferspeed(1);
                //shooter.setKickerspeed(0);
                // }
            }

            if (currentGamepad1.circle && !previousGamepad1.circle) {
                // Change to gamepad2 if that's what you need.
                shooter.setshooter(1);
                //transferTime2.reset();
                // if(transferTime2.milliseconds() >= 1500){
                //intake.setTransferspeed(1);
                //shooter.setKickerspeed(0);
                // }
            }

            if (currentGamepad1.square && !previousGamepad1.square) {
                // Change to gamepad2 if that's what you need.
                shooter.setshooter(-1);
                transferTime2.reset();
                // if(transferTime2.milliseconds() >= 1500){
                intake.setTransferspeed(0);
                // }



            }

            if (currentGamepad1.cross && !previousGamepad1.cross) {  // Change to gamepad2 if that's what you need.
                shooter.setshooter(0);
                shooter.setKickerspeed(0.5);
                intake.setTransferspeed(0.5);

            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {  // Change to gamepad2 if that's what you need.
                shooter.setKickerspeed(0);
                intake.setTransferspeed(1);

            } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {  // Change to gamepad2 if that's what you need.
                shooter.setKickerspeed(1);
                intake.setTransferspeed(1);
            } else if(currentGamepad1.leftBumperWasReleased() || currentGamepad1.rightBumperWasReleased()) {
                shooter.setKickerspeed(0.5);
            }

           /// ???????
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {  // Change to gamepad2 if that's what you need.
               turret.setturretspeed(1);

            } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {  // Change to gamepad2 if that's what you need.
                turret.setturretspeed(-1);
            } else if(currentGamepad1.leftBumperWasReleased() || currentGamepad1.rightBumperWasReleased()) {
                turret.setturretspeed(0.5);
            }
            ////?????


            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {  // Change to gamepad2 if that's what you need.
                shooter.setPitchPosition(shooter.getPitchPosition()+0.05);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {  // Change to gamepad2 if that's what you need.
                shooter.setPitchPosition(shooter.getPitchPosition()-0.05);
            }

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {  // Change to gamepad2 if that's what you need.
                intake.setblockposition(intake.gateup);
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {  // Change to gamepad2 if that's what you need.
                intake.setblockposition(intake.gatedown);
            }

            switch (intake.getIntakeState()) {
                case IDLE:
                    break;
                case INTAKING:
                    intake.setTransferspeed(1);// Change to gamepad2 if that's what you need.
                    intake.setSpinner(-1);
                    //intake.setIntakeState(IntakeState.SCANNING);
                    break;
                case STOP_SPINNER:
                    intake.setSpinner(0);
                    //intake.setIntakeState(IntakeState.SCANNING);
                    break;

            }

            switch (intake.getTransferState()) {
                case IDLE:
                    break;
                case SCANNING:
                    if(!intake.breakBeamBoolean()){
                        intake.setTransferspeed(0.5);
                        intake.setTransferState(TransferState.TRANSFEROFF);
                    }
                    //intake.setIntakeState(IntakeState.SCANNING);
                    break;
                case TRANSFEROFF:
                    intake.setTransferspeed(0.5);
                    //intake.setIntakeState(IntakeState.SCANNING);
                    break;

            }

            updateTelemetry();
        }

    }




    /**
     * Sends debug data to control hub
     */
    private void updateTelemetry () {
       // telemetry.addData("Intake Data", 1);
        telemetry.addData("pitch_pos:", shooter.getPitchPosition());
        telemetry.addData("kickerspeed:", shooter.getKickerPosition());
        //telemetry.addData("transferclock:", transferTime2.seconds());
        telemetry.addData("shooter:", shooter.getLrhino().getPower());
        telemetry.addData("DISTANCESENSOR", intake.ballthreeinrobot());
        telemetry.addData("breakbeam detected:", intake.breakBeamBoolean());
        telemetry.addData("intakestate:", intake.getIntakeState());
        telemetry.addData("transferstate:", intake.getTransferState());
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
        shooter.init(hardwareMap);

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


}
