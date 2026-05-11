package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 * read me FIRST
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * HARDWARE:
 *   - 2x goBILDA 5000 Series 12VDC Brushed Motor (8mm REX® Pinion Shaft)
 *       Encoder: 28 ticks/rev (no gearbox) | Measured loaded max: 2460 ticks/sec
 *   - 2x goBILDA 3614 Series Rhino Wheel (72mm diameter, 30A Durometer)
 *   - 4x Steel Flywheel (60mm, 115g, 551 g·cm² each) — total inertia: 2204 g·cm²
 *
 * NOTE: Only launch1 encoder is connected. Both PIDF controllers use
 *       launcherLeft (launch1) velocity as feedback.
 *
 * ⚠️  BEFORE YOU TUNE — READ THIS:
 *   - Always tune on a FULL battery (13.0–13.8V). kF tuned at 12V will be wrong
 *     at 13.5V. Voltage directly scales motor output. Be consistent every session.
 *   - 28 ticks/rev is very coarse. Velocity readings will look noisy/steppy on the
 *     graph — that is normal. Keep kD = 0 always. Derivative on 28 tick resolution
 *     is pure noise amplification.
 *   - Do NOT set TARGET_VELOCITY above 2300. Measured loaded max is 2460 ticks/sec.
 *     Commanding above physical max causes permanent error → integral windup →
 *     motor pegged at 100% power → runs hot, controller behaves erratically.
 *   - FLOAT zero power behavior is intentional. BRAKE would fight the flywheel
 *     inertia and stress the motor on stop.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * HOW SolversLib PIDF WORKS HERE
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   calculate(measuredVelocity, targetVelocity) runs every loop and returns
 *   a motor power value in [-1, 1]. You call setPower() with that value.
 *   This is different from SDK's setVelocity() — you are fully in control.
 *
 *   kF (Feedforward) — Most important. Get this right first.
 *       output += kF * targetVelocity
 *       Formula: kF = desired_power / target_velocity
 *       Example: 0.82 power at 1800 ticks/sec → kF = 0.82/1800 = 0.000456
 *
 *   kP (Proportional) — Corrects remaining error after kF.
 *       Too low = slow recovery. Too high = oscillation.
 *
 *   kI (Integral) — Fixes persistent steady-state error kP can't close.
 *       Use tiny values only. reset() clears windup on spin-up.
 *
 *   kD (Derivative) — DO NOT USE. 28 ticks/rev = pure noise.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * SETUP
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   1. Charge battery to full (13.0V+).
 *   2. Connect to robot Wi-Fi
 *   3. Open http://192.168.43.1:8080/dash
 *   4. Run this OpMode
 *   5. Graph: targetVelocity, velocity (launch1)
 *   6. Variable Configuration panel → find "LauncherTuning"
 *      ⚠️  After typing a new value, press ENTER to commit it.
 *          Verify the kP/kI/kD/kF telemetry line updates — if it does,
 *          the gains are live. If not, something is wrong with @Config.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * TUNING PROCEDURE — FOLLOW IN ORDER
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   STEP 1 — TUNE kF FIRST
 *   STEP 1 — TUNE kF FIRST
 *   ─────────────────────────────────────────────────────────
 *   Set: kP=0, kI=0, kD=0, kF=0.00038, TARGET_VELOCITY=1800
 *   Press Y. Watch where motor settles. Read rightPower from telemetry.
 *   Recalculate: kF = rightPower / actual_settled_velocity
 *   Goal: settle within ~150 ticks/sec BELOW target on kF alone.
 *   kF should undershoot slightly — kP closes the rest.
 *   Press B between each adjustment. Let flywheel fully stop before Y again.
 *
 *   STEP 2 — TUNE kP
 *   ─────────────────────────────────────────────────────────
 *   Start kP=0.0003. Raise in steps: 0.0003→0.0006→0.0009→0.0012
 *   Want: fast snap to target, flat line, no oscillation.
 *   Too high kP: velocity bounces above/below target.
 *   Too low kP: slow lazy climb, never quite settles.
 *
 *   STEP 3 — TEST UNDER LOAD
 *   ─────────────────────────────────────────────────────────
 *   Press ball against Rhino wheels. Watch dip and recovery.
 *   Good: dip <200 ticks/sec, recovery <0.3s, no oscillation after.
 *   Deep dip + slow recovery → raise kP
 *   Oscillation after recovery → lower kP, recheck kF
 *
 *   STEP 4 — ADD kI ONLY IF NEEDED
 *   ─────────────────────────────────────────────────────────
 *   Persistent gap after Steps 1-3? Add kI: start 0.000005, max ~0.00003.
 *   Windup sign: power keeps climbing after reaching target → kI too high.
 *
 *   STEP 5 — REPEAT AT MATCH VELOCITY
 *   ─────────────────────────────────────────────────────────
 *   Use LB/LT to set match TARGET_VELOCITY. Re-verify tune holds there.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * OPTIONAL — SquIDF INSTEAD OF PIDF
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   If spin-up consistently overshoots, try SquIDFController (square-root PIDF).
 *   Smoother spin-up — √(error) dampens aggressive corrections at large error.
 *   Import: com.seattlesolvers.solverslib.controller.SquIDFController
 *   Drop-in replacement — same calculate(measured, target) API.
 *   kP values will need to be higher since √(e) < e for errors > 1.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * CONTROLS (gamepad1)
 * ═══════════════════════════════════════════════════════════════════════════
 *   Y  — Spin up launcher    |  B  — Stop launcher
 *   LB — Target +STEP        |  LT — Target -STEP
 *   A  — Toggle intake + feeder
 */

@Config
@TeleOp(name = "Launcher PIDF Tuner", group = "Tuning")
public class LauncherTuning extends OpMode {

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00038;

    public static double TARGET_VELOCITY = 2000;
    public static double STEP            = 50;

    private static final double MAX_TICKS_SEC   = 2460;
    private static final double MAX_SAFE_TARGET = 2300;

    private Motor intake, feeder;
    private GamepadEx gamepadEx;
    private boolean intakeRunning = false;

    private DcMotorEx launcherRight, launcherLeft;
    private PIDFController pidfRight, pidfLeft;

    private boolean running = false;
    private boolean lastY = false, lastB = false, lastLB = false, lastLT = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard.getInstance().withConfigRoot(config ->
                config.putVariable(getClass().getSimpleName(),
                        ReflectionConfig.createVariableFromClass(LauncherTuning.class)));

        gamepadEx = new GamepadEx(gamepad1);

        intake = new Motor(hardwareMap, "intake");
        feeder = new Motor(hardwareMap, "feeder");
        intake.setRunMode(Motor.RunMode.RawPower);
        feeder.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launch1");
        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidfRight = new PIDFController(kP, kI, kD, kF);
        pidfLeft  = new PIDFController(kP, kI, kD, kF);

        telemetry.addData("Status", "Ready. Y=spin up | B=stop | LB/LT=adjust target | A=toggle intake");
        telemetry.addData("Max safe target", MAX_SAFE_TARGET + " ticks/sec (loaded max " + MAX_TICKS_SEC + ")");
        telemetry.addData("Encoder", "Using launch1 only");
        telemetry.update();
    }

    @Override
    public void loop() {
        pidfRight.setPIDF(kP, kI, kD, kF);
        pidfLeft.setPIDF(kP, kI, kD, kF);

        gamepadEx.readButtons();

        intake.set(1);

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            intakeRunning = !intakeRunning;
            if (intakeRunning) {
                feeder.set(1);
            } else {
                feeder.set(0);
            }
        }

        boolean y  = gamepad1.y;
        boolean b  = gamepad1.b;
        boolean lb = gamepad1.left_bumper;
        boolean lt = gamepad1.left_trigger > 0.1;

        if (y && !lastY) {
            running = true;
            pidfRight.reset();
            pidfLeft.reset();
        }
        if (b && !lastB) {
            running = false;
            launcherRight.setPower(0);
            launcherLeft.setPower(0);
        }
        if (lb && !lastLB) TARGET_VELOCITY = Math.min(TARGET_VELOCITY + STEP, MAX_SAFE_TARGET);
        if (lt && !lastLT) TARGET_VELOCITY = Math.max(TARGET_VELOCITY - STEP, 0);

        lastY = y; lastB = b; lastLB = lb; lastLT = lt;

        // Only launch1 has an encoder — use it as feedback for both controllers
        double velocity = launcherRight.getVelocity();

        if (running) {
            double rightPower = pidfRight.calculate(velocity, TARGET_VELOCITY);
            double leftPower  = pidfLeft.calculate(velocity,  TARGET_VELOCITY);

            rightPower = Math.max(0, Math.min(1, rightPower));
            leftPower  = Math.max(0, Math.min(1, leftPower));

            launcherRight.setPower(rightPower);
            launcherLeft.setPower(leftPower);

            telemetry.addData("rightPower", rightPower);
            telemetry.addData("leftPower",  leftPower);
        }

        telemetry.addData("intake",         intakeRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("status",         running ? "RUNNING" : "STOPPED");
        telemetry.addData("targetVelocity", TARGET_VELOCITY);
        telemetry.addData("velocity (launch)", velocity);
        telemetry.addData("error",          TARGET_VELOCITY - velocity);
        telemetry.addData("kP/kI/kD/kF",   kP + " / " + kI + " / " + kD + " / " + kF);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.set(0);
        feeder.set(0);
        launcherRight.setPower(0);
        launcherLeft.setPower(0);
    }
}