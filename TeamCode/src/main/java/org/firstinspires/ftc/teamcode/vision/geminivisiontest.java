package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.hardware.turret;

/**
 * Manages all Limelight vision tracking, encoder limits, and movement commands for the turret.
 * Assumes the turret is controlled by a Continuous Rotation Servo and tracked by a DcMotorEx encoder.
 */
public class geminivisiontest {

    // --- HARDWARE DECLARATIONS ---
    private Servo turretCRServo;
    private DcMotorEx encoderReader;
    private Limelight3A limelight;
    private org.firstinspires.ftc.teamcode.hardware.turret turret;

    // --- ENCODER LIMITS & CONVERSION CONSTANTS (TUNE THESE BASED ON YOUR ROBOT) ---
    private final int MAX_TICK_LIMIT = 4000;      // Max positive tick position
    private final int MIN_TICK_LIMIT = -4000;     // Max negative tick position
    private final int CENTER_TICK_POSITION = 0;   // Assumes home position is 0 ticks
    private final int TICKS_PER_DEGREE = 80;      // Conversion factor: Limelight degrees to Encoder ticks

    // --- TRACKING CONSTANTS ---
    private final double TARGET_TOLERANCE_DEGREES = 1.0; // Deadband (how centered is acceptable)
    private final double TARGET_AREA_THRESHOLD = 0.01;   // Minimum Limelight Area (ta) for target visibility

    // --- MOVEMENT GAINS (Used internally by moveTurretToPosition if external call is not used) ---
    private final double KP_MOVE = 0.0001;
    private final double SPEED_LIMIT = 0.3;

    /**
     * Initializes all turret hardware and resets the encoder.
     *
     * @param hwMap The robot's HardwareMap.
     */
    public geminivisiontest(HardwareMap hwMap) {
        this.turret = new turret();
        turret.init(hwMap);
    }

    // --- CORE TRACKING METHOD ---

    /**
     * Executes the Limelight AprilTag tracking logic to find the target tick position
     * and calls moveTurretToPosition to drive the servo.
     *
     * @return String status of the tracking operation for telemetry.
     */
    public String trackAprilTag() {
        LLResult result = limelight.getLatestResult();
        boolean targetVisible = result != null && result.isValid() && (result.getTa() > TARGET_AREA_THRESHOLD);

        int currentTicks = encoderReader.getCurrentPosition();

        if (targetVisible) {

            double tx = result.getTx(); // Horizontal offset (error in degrees)

            // 1. Calculate Target Tick Position

            // Convert Limelight's angular error (tx) into a tick correction
            int tickCorrection = (int) (tx * TICKS_PER_DEGREE);

            // Target Ticks = Center Position + Correction
            int targetTicks = CENTER_TICK_POSITION + tickCorrection;

            // 2. Clamp Target to Absolute Limits
            targetTicks = Math.min(targetTicks, MAX_TICK_LIMIT);
            targetTicks = Math.max(targetTicks, MIN_TICK_LIMIT);

            // 3. Command Movement or Stop
            if (Math.abs(tx) > TARGET_TOLERANCE_DEGREES) {
                // Call the existing function to move to the calculated position
               // this.moveTurretToPosition(targetTicks);
                return String.format("Tracking | Target TX: %.2f | Ticks: %d", tx, currentTicks);
            } else {
                // Target is centered, stop the turret
               // this.moveTurretToPosition(currentTicks); // Move to current position (which stops it)
                return String.format("Target Centered | Ticks: %d", currentTicks);
            }

        } else {
            // No target found. Stop the servo.
            turretCRServo.setPosition(0.5);
            return String.format("No Target Found | Ticks: %d", currentTicks);
        }
    }

    public turret getTurret () {
        return turret;
    }
}