package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;

/**
 * LAUNCHER PIDF TUNING OPMODE — SolversLib + FTC Dashboard
 *
 * Hardware:
 *   - 2x goBILDA 5000 Series 12VDC Brushed Motor (8mm REX® Pinion Shaft)
 *       Tested free-spin: ~5800 RPM | Encoder: 28 ticks/rev (no gearbox)
 *       Max ticks/sec ≈ 2707
 *   - 2x goBILDA 3614 Series Rhino Wheel (72mm, 30A Durometer)
 *   - 4x Steel Flywheel (60mm, 115g, 551 g·cm² each) — total inertia: 2204 g·cm²
 *
 * HOW SolversLib PIDF WORKS HERE:
 *   - calculate(measuredVelocity, targetVelocity) runs full PIDF each loop
 *   - kF term: output += kF * setpoint (pure feedforward scaled to motor power)
 *   - Output is in [-1, 1] motor power range → fed directly into setPower()
 *   - Controller timestamps itself internally — loop time is accounted for automatically
 *
 * ⚠️  28 ticks/rev = coarse velocity readings. Keep kD = 0.
 *     Do NOT set TARGET_VELOCITY above ~2400 ticks/sec (hard max ~2707).
 *
 * TUNING STEPS:
 *   1. kP=0, kI=0, kD=0, kF=0.00040 → Press Y
 *      Motor should reach ~70-80% of target speed on feedforward alone.
 *      Nudge kF until it lands close (kF = desired_power / target_velocity).
 *      Example: want 0.80 power at 1800 ticks/sec → kF = 0.80/1800 ≈ 0.00044
 *   2. Raise kP (try 0.0003 → 0.0006 → 0.001) until it snaps to target without oscillating
 *      Heavy flywheel inertia = you'll likely need higher kP than expected
 *   3. Fire balls, watch graph — good tune = small dip on contact, fast recovery (<0.3s)
 *   4. If steady-state error persists after kP, add tiny kI (0.00001 increments).
 *      Watch for windup — reset() clears it on each spin-up.
 *
 * CONTROLS (gamepad1):
 *   Y  — Spin up  |  B  — Stop
 *   LB — Target +STEP  |  LT — Target -STEP
 */

@Config
@TeleOp(name = "Launcher PIDF Tuner", group = "Tuning")
public class LauncherTuningSir extends OpMode {

    // Edit live in FTC Dashboard "Variable Configuration" panel
    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.0;     // Keep 0 — 28 tick/rev makes derivative useless
    public static double kF = 0.00044; // kF * targetVelocity ≈ base motor power

    public static double TARGET_VELOCITY = 1800; // ticks/sec — do NOT exceed ~2400
    public static double STEP            = 50;

    private DcMotorEx launcherRight, launcherLeft;
    private PIDFController pidfRight, pidfLeft;

    private boolean running = false;
    private boolean lastY = false, lastB = false, lastLB = false, lastLT = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launch1");

        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        // RUN_WITHOUT_ENCODER — we control power directly, encoder still reads velocity
        launcherRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidfRight = new PIDFController(kP, kI, kD, kF);
        pidfLeft  = new PIDFController(kP, kI, kD, kF);

        telemetry.addData("Status", "Ready. Y=spin up | B=stop | LB/LT=adjust target");
        telemetry.addData("Max safe target", "~2400 ticks/sec (hard max ~2707)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Push dashboard gain changes into controllers live every loop
        pidfRight.setPIDF(kP, kI, kD, kF);
        pidfLeft.setPIDF(kP, kI, kD, kF);

        boolean y  = gamepad1.y;
        boolean b  = gamepad1.b;
        boolean lb = gamepad1.left_bumper;
        boolean lt = gamepad1.left_trigger > 0.1;

        if (y  && !lastY) {
            running = true;
            pidfRight.reset(); // Clear integral windup on every spin-up
            pidfLeft.reset();
        }
        if (b  && !lastB) {
            running = false;
            launcherRight.setPower(0);
            launcherLeft.setPower(0);
        }
        if (lb && !lastLB) TARGET_VELOCITY = Math.min(TARGET_VELOCITY + STEP, 2400);
        if (lt && !lastLT) TARGET_VELOCITY = Math.max(TARGET_VELOCITY - STEP, 0);

        lastY = y; lastB = b; lastLB = lb; lastLT = lt;

        double rightVelocity = launcherRight.getVelocity();
        double leftVelocity  = launcherLeft.getVelocity();

        if (running) {
            // calculate(measuredValue, setpoint) — correct SolversLib API pattern
            double rightPower = pidfRight.calculate(rightVelocity, TARGET_VELOCITY);
            double leftPower  = pidfLeft.calculate(leftVelocity,  TARGET_VELOCITY);

            // Safety clamp — flywheel should only ever spin forward
            rightPower = Math.max(0, Math.min(1, rightPower));
            leftPower  = Math.max(0, Math.min(1, leftPower));

            launcherRight.setPower(rightPower);
            launcherLeft.setPower(leftPower);

            telemetry.addData("rightPower", rightPower);
            telemetry.addData("leftPower",  leftPower);
        }

        // Graph targetVelocity + rightVelocity + leftVelocity in Dashboard
        telemetry.addData("status",         running ? "RUNNING" : "STOPPED");
        telemetry.addData("targetVelocity", TARGET_VELOCITY);
        telemetry.addData("rightVelocity",  rightVelocity);
        telemetry.addData("leftVelocity",   leftVelocity);
        telemetry.addData("rightError",     TARGET_VELOCITY - rightVelocity);
        telemetry.addData("leftError",      TARGET_VELOCITY - leftVelocity);
        telemetry.addData("kP/kI/kD/kF",   kP + " / " + kI + " / " + kD + " / " + kF);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcherRight.setPower(0);
        launcherLeft.setPower(0);
    }
}