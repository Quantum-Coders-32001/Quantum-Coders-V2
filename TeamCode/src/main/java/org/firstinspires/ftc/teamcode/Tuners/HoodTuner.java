package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

/**
 * ═══════════════════════════════════════════════════════════════════════
 * HOOD REGRESSION TUNER
 * ═══════════════════════════════════════════════════════════════════════
 *
 * PURPOSE:
 *   Shoot from multiple distances and collect (distance → hood position)
 *   pairs. Everything tunable from FTC Dashboard — no touching code.
 *
 * HOW TO COLLECT DATA:
 *   1. Open Dashboard → http://192.168.43.1:8080/dash
 *   2. Run OpMode. Set START_X/Y/HEADING to your robot's start position.
 *   3. Drive to a shooting spot (gamepad 1).
 *   4. In Dashboard: set HOOD_POSITION and LAUNCHER_TARGET_VELOCITY.
 *   5. Press Y (gamepad 2) to spin up launcher.
 *   6. Press A to toggle intake, RB to toggle feeder and shoot.
 *   7. Adjust HOOD_POSITION until balls go in consistently.
 *   8. Read "Record This Pair" from telemetry — write it down.
 *   9. Press Y again to stop launcher, drive to next spot, repeat.
 *   10. Paste pairs into Desmos → y₁ ~ ax₁² + bx₁ + c
 *
 * DASHBOARD VARIABLES:
 *   HOOD_POSITION              servo position to hold (0.0 – 1.0)
 *   LAUNCHER_TARGET_VELOCITY   target flywheel velocity (ticks/sec)
 *   kP / kI / kD / kF         PIDF coefficients for launcher
 *   GOAL_X / GOAL_Y            goal coordinate in inches (Pedro coords)
 *   START_X / START_Y / START_HEADING   robot starting pose for Pinpoint
 *
 * GAMEPAD 1 — drive:
 *   Left stick        forward / strafe
 *   Right stick X     rotate
 *
 * GAMEPAD 2 — shooter:
 *   Y     spin up / stop launcher
 *   A     toggle intake on/off
 *   RB    toggle feeder on/off
 *
 * HARDWARE NAMES (match your existing config):
 *   lf, rf, lb, rb    drivetrain
 *   launch, launch1   launcher motors (DcMotorEx)
 *   intake            intake motor
 *   feeder            feeder motor
 *   tilt              hood tilt servo
 *   pinpoint          GoBILDA Pinpoint (via Constants.createFollower)
 *
 * ═══════════════════════════════════════════════════════════════════════
 */
@Config
@TeleOp(name = "Hood Tuner", group = "Tuning")
public class HoodTuner extends OpMode {

    // ─── Pinpoint starting pose ──────────────────────────────────────────
    public static double START_X       = 12.0;   // inches, Pedro x = forward
    public static double START_Y       = 12.0;   // inches, Pedro y = lateral
    public static double START_HEADING = 45.0;   // degrees

    // ─── Goal coordinate (red close-alliance) ────────────────────────────
    // x=68, y=68 matches FieldConstants.GoalCoordinates.RED in ChaiGPT's repo
    public static double GOAL_X = 68.0;
    public static double GOAL_Y = 68.0;

    // ─── Hood position (edit in Dashboard) ───────────────────────────────
    public static double HOOD_POSITION = 0.5;
    public static double MIN_POSITION  = 0.07;
    public static double MAX_POSITION  = 0.70;

    // ─── Launcher PIDF (edit in Dashboard) ───────────────────────────────
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00038;

    public static double LAUNCHER_TARGET_VELOCITY = 1200;
    public static double VELOCITY_TOLERANCE       = 75;
    private static final double MAX_SAFE_VELOCITY = 2300;

    // ─── Hardware ────────────────────────────────────────────────────────
    private DcMotor   lf, rf, lb, rb;
    private DcMotorEx launcherRight, launcherLeft;
    private Motor     intake, feeder;
    private Servo     tiltServo;
    private Follower  follower;

    // ─── SolversLib ──────────────────────────────────────────────────────
    private PIDFController pidfRight, pidfLeft;
    private GamepadEx      gp1, gp2;

    // ─── Launcher state machine ───────────────────────────────────────────
    private enum LaunchState { IDLE, SPIN_UP, RUNNING }
    private LaunchState launchState = LaunchState.IDLE;

    // ─── Toggle state ─────────────────────────────────────────────────────
    private boolean intakeRunning = false;
    private boolean feederRunning = false;

    private MultipleTelemetry telem;

    // ════════════════════════════════════════════════════════════════════
    @Override
    public void init() {

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Register @Config variables so Dashboard can edit them live
        FtcDashboard.getInstance().withConfigRoot(config ->
                config.putVariable(getClass().getSimpleName(),
                        ReflectionConfig.createVariableFromClass(HoodTuner.class)));

        // ── SolversLib gamepads ─────────────────────────────────────────
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        // ── Drivetrain ──────────────────────────────────────────────────
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Launcher ────────────────────────────────────────────────────
        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launch1");

        launcherRight.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidfRight = new PIDFController(kP, kI, kD, kF);
        pidfLeft  = new PIDFController(kP, kI, kD, kF);

        // ── Intake & feeder (SolversLib Motor) ──────────────────────────
        intake = new Motor(hardwareMap, "intake");
        feeder = new Motor(hardwareMap, "feeder");
        intake.setRunMode(Motor.RunMode.RawPower);
        feeder.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // ── Hood servo ──────────────────────────────────────────────────
        tiltServo = hardwareMap.get(Servo.class, "tilt");
        tiltServo.setPosition(clampHood(HOOD_POSITION));

        // ── Pinpoint / Pedro follower ────────────────────────────────────
        follower = org.firstinspires.ftc.teamcode.tuners.Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING)));

        telem.addLine("Hood Tuner ready.");
        telem.addLine("Edit HOOD_POSITION & LAUNCHER_TARGET_VELOCITY in Dashboard.");
        telem.addLine("GP2: Y = launcher on/off  |  A = intake  |  RB = feeder");
        telem.update();
    }

    // ════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {

        // ── Read gamepads ────────────────────────────────────────────────
        gp1.readButtons();
        gp2.readButtons();

        // ── Update Pinpoint odometry ─────────────────────────────────────
        follower.update();

        // ── Sync PIDF coefficients from Dashboard every loop ─────────────
        pidfRight.setPIDF(kP, kI, kD, kF);
        pidfLeft.setPIDF(kP, kI, kD, kF);

        // ── Drivetrain (gamepad 1) ───────────────────────────────────────
        mecanumDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);

        // ── Hood servo — Dashboard controls position ─────────────────────
        tiltServo.setPosition(clampHood(HOOD_POSITION));

        // ── Intake toggle (gamepad 2 A) ──────────────────────────────────
        if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
            intakeRunning = !intakeRunning;
        }
        intake.set(intakeRunning ? 1.0 : 0.0);

        // ── Feeder toggle (gamepad 2 RB) ─────────────────────────────────
        if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            feederRunning = !feederRunning;
        }
        feeder.set(feederRunning ? 1.0 : -0.3);

        // ── Launcher state machine (gamepad 2 Y) ─────────────────────────
        if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
            if (launchState == LaunchState.IDLE) {
                pidfRight.reset();
                pidfLeft.reset();
                launchState = LaunchState.SPIN_UP;
            } else {
                stopLauncher();
                launchState = LaunchState.IDLE;
            }
        }

        double safeTarget = Math.min(LAUNCHER_TARGET_VELOCITY, MAX_SAFE_VELOCITY);
        double rightVel   = launcherRight.getVelocity();
        double leftVel    = launcherLeft.getVelocity();

        switch (launchState) {
            case IDLE:
                break;

            case SPIN_UP:
                runPIDF(safeTarget, rightVel, leftVel);
                if (Math.abs(rightVel - safeTarget) < VELOCITY_TOLERANCE
                        && Math.abs(leftVel  - safeTarget) < VELOCITY_TOLERANCE) {
                    launchState = LaunchState.RUNNING;
                }
                break;

            case RUNNING:
                runPIDF(safeTarget, rightVel, leftVel);
                break;
        }

        // ── Distance to goal (inline — no external class needed) ──────────
        Pose   robotPose = follower.getPose();
        double dx        = GOAL_X - robotPose.getX();
        double dy        = GOAL_Y - robotPose.getY();
        double distance  = Math.sqrt(dx * dx + dy * dy);

        // ── Telemetry ────────────────────────────────────────────────────
        telem.addLine("═══ Hood Regression Tuner ═══");
        telem.addData("Hood Position",    "%.4f", clampHood(HOOD_POSITION));
        telem.addData("Distance to Goal", "%.2f in  (%.1f ft)",
                distance, distance / 12.0);
        telem.addLine("");
        telem.addLine("─── Record This Pair ───");
        telem.addData("x₁ (distance)", "%.2f", distance);
        telem.addData("y₁ (position)", "%.4f", clampHood(HOOD_POSITION));
        telem.addLine("  Desmos: y₁ ~ ax₁² + bx₁ + c");
        telem.addLine("");
        telem.addLine("─── Launcher ───");
        telem.addData("State",      launchState);
        telem.addData("Target Vel", "%.0f ticks/s", safeTarget);
        telem.addData("Right Vel",  "%.0f", rightVel);
        telem.addData("Left Vel",   "%.0f", leftVel);
        telem.addData("At Speed",   Math.abs(rightVel - safeTarget) < VELOCITY_TOLERANCE
                && Math.abs(leftVel  - safeTarget) < VELOCITY_TOLERANCE);
        telem.addData("kP/kI/kD/kF", kP + " / " + kI + " / " + kD + " / " + kF);
        telem.addLine("");
        telem.addLine("─── Subsystems ───");
        telem.addData("Intake",  intakeRunning ? "ON" : "OFF");
        telem.addData("Feeder",  feederRunning ? "ON" : "OFF");
        telem.addLine("");
        telem.addLine("─── Robot Pose (Pinpoint) ───");
        telem.addData("x",       "%.2f in", robotPose.getX());
        telem.addData("y",       "%.2f in", robotPose.getY());
        telem.addData("heading", "%.1f°",   Math.toDegrees(robotPose.getHeading()));
        telem.update();
    }

    // ════════════════════════════════════════════════════════════════════
    @Override
    public void stop() {
        stopLauncher();
        intake.set(0);
        feeder.set(0);
    }

    // ── Helpers ──────────────────────────────────────────────────────────
    private void runPIDF(double target, double rightVel, double leftVel) {
        double rPow = pidfRight.calculate(rightVel, target);
        double lPow = pidfLeft.calculate(leftVel,  target);
        launcherRight.setPower(Math.max(0, Math.min(1, rPow)));
        launcherLeft.setPower(Math.max(0,  Math.min(1, lPow)));
    }

    private void stopLauncher() {
        launcherRight.setPower(0);
        launcherLeft.setPower(0);
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        double denom = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);
        lf.setPower((forward + strafe + rotate) / denom);
        rf.setPower((forward - strafe - rotate) / denom);
        lb.setPower((forward - strafe + rotate) / denom);
        rb.setPower((forward + strafe - rotate) / denom);
    }

    private double clampHood(double pos) {
        return Math.max(MIN_POSITION, Math.min(MAX_POSITION, pos));
    }
}