package org.firstinspires.ftc.teamcode.tuners;

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

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretAiming;

/**
 * ═══════════════════════════════════════════════════════════════════════
 * HOOD REGRESSION TUNER
 * ═══════════════════════════════════════════════════════════════════════
 *
 * HOW TO COLLECT DATA:
 *   1. Open Dashboard → http://192.168.43.1:8080/dash
 *   2. Set START_X / START_Y / START_HEADING to where you place the robot.
 *   3. Drive to a shooting spot (gamepad 1 sticks).
 *   4. In Dashboard: adjust HOOD_POSITION and LAUNCHER_TARGET_VELOCITY.
 *   5. Press Y (gamepad 2) to spin up launcher.
 *   6. Press A to toggle intake, RB to toggle feeder — shoot.
 *   7. Tune HOOD_POSITION in Dashboard until balls go in consistently.
 *   8. Write down the "Record This Pair" values from telemetry.
 *      → Telemetry tells you which regression set this point belongs to
 *        (CLOSE side: robot y > 48, FAR side: robot y < 48)
 *   9. Press Y again to stop, drive to next distance, repeat.
 *   10. Paste each set into Desmos separately:
 *         CLOSE side pairs → y₁ ~ ax₁² + bx₁ + c   (quadratic regression)
 *         FAR   side pairs → y₁ ~ ax₁² + bx₁ + c   (quadratic regression)
 *       You get two separate sets of a/b/c coefficients.
 *
 * WHY TWO REGRESSIONS?
 *   In Pedro Pathing coords the field geometry changes depending on which
 *   side of y=48 the robot is on — the angle to the goal and the physical
 *   trajectory of the ball are different enough that one curve fits both
 *   sides poorly. Splitting at y=48 lets each regression fit its region well.
 *   Use quadratic (not linear) because launch angle vs distance is curved.
 *
 * DASHBOARD VARIABLES (all editable live):
 *   HOOD_POSITION              tilt servo position (0.0 – 1.0)
 *   LAUNCHER_TARGET_VELOCITY   flywheel target (ticks/sec)
 *   GOAL_X / GOAL_Y            goal coordinate (Pedro inches)
 *   START_X / START_Y / START_HEADING   Pinpoint starting pose
 *   (Launcher PIDF gains come from Constants.kp / Constants.kf — edit there)
 *
 * GAMEPAD 1 — drive only:
 *   Left stick        forward / strafe
 *   Right stick X     rotate
 *
 * GAMEPAD 2 — shooter:
 *   Y     spin up / stop launcher (toggle)
 *   A     toggle intake on/off
 *   RB    toggle feeder on/off
 *
 * HARDWARE NAMES (managed by subsystems):
 *   lf, rf, lb, rb    drivetrain      → DrivetrainSubsystem
 *   lturret, rturret  turret servos   → TurretAiming
 *   rf                turret encoder  → TurretAiming (separate encoder cable)
 *   launch, launch1   launcher (DcMotorEx — encoder on launch only)
 *   intake            intake (SolversLib Motor)
 *   feeder            feeder (SolversLib Motor)
 *   tilt              hood servo
 *   pinpoint          GoBILDA Pinpoint (via Constants.createFollower)
 *
 * ═══════════════════════════════════════════════════════════════════════
 */
@Config
@TeleOp(name = "Hood Tuner", group = "Tuning")
public class HoodTuner extends OpMode {

    // ─── Pinpoint starting pose ──────────────────────────────────────────
    public static double START_X       = 0.0;
    public static double START_Y       = 0.0;
    public static double START_HEADING = 0.0;

    // ─── Goal coordinate (red close-alliance, Pedro coords) ──────────────
    public static double GOAL_X = 143.46949602122018;
    public static double GOAL_Y = 136.2652519893899;

    // ─── Hood ────────────────────────────────────────────────────────────
    public static double HOOD_POSITION = 0.5;
    public static double MIN_POSITION  = 0.07;
    public static double MAX_POSITION  = 0.70;

    // ─── Launcher PIDF — values from Constants.kp / Constants.kf ────────
    // To change gains, edit Constants.java and redeploy. kI and kD are 0.
    public static double LAUNCHER_TARGET_VELOCITY = 1200;
    public static double VELOCITY_TOLERANCE       = 75;

    private static final double MAX_SAFE_VELOCITY = 2300;
    private static final double MAX_TICKS_SEC     = 2460;

    // ─── Regression split ────────────────────────────────────────────────
    // robot y > 48 = CLOSE side, robot y < 48 = FAR side (Pedro coords)
    private static final double REGRESSION_SPLIT_Y = 48.0;

    // ─── Subsystems ──────────────────────────────────────────────────────
    private DrivetrainSubsystem drivetrain;
    private TurretAiming        turret;
    private Follower            follower;

    // ─── Launcher hardware ───────────────────────────────────────────────
    private com.qualcomm.robotcore.hardware.DcMotorEx launcherRight, launcherLeft;

    // ─── Intake / feeder / hood ──────────────────────────────────────────
    private Motor intake, feeder;
    private Servo tiltServo;

    // ─── SolversLib ──────────────────────────────────────────────────────
    private GamepadEx      gamepadEx;
    private PIDFController pidfRight, pidfLeft;

    // ─── State ───────────────────────────────────────────────────────────
    private boolean running       = false;
    private boolean intakeRunning = false;
    private boolean feederRunning = false;

    // ════════════════════════════════════════════════════════════════════
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard.getInstance().withConfigRoot(config ->
                config.putVariable(getClass().getSimpleName(),
                        ReflectionConfig.createVariableFromClass(HoodTuner.class)));

        // ── Gamepad ──────────────────────────────────────────────────────
        gamepadEx = new GamepadEx(gamepad2);

        // ── Pinpoint via Constants ───────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING)));

        // ── Drivetrain subsystem ─────────────────────────────────────────
        // All motor setup (directions, brake, run mode) is handled inside
        // DrivetrainSubsystem — no manual motor init needed here.
        drivetrain = new DrivetrainSubsystem(hardwareMap);

        // ── Turret subsystem ─────────────────────────────────────────────
        // Owns lturret / rturret CRServos and the rf encoder port.
        // NOTE: rf encoder is reset inside TurretAiming — do NOT touch it here.
        turret = new TurretAiming(hardwareMap, follower);
        turret.setTarget(new Pose(GOAL_X, GOAL_Y));
        turret.enable();

        // ── Launcher ─────────────────────────────────────────────────────
        launcherRight = hardwareMap.get(
                DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(
                DcMotorEx.class, "launch1");

        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidfRight = new PIDFController(Constants.kp, 0, 0, Constants.kf);
        pidfLeft  = new PIDFController(Constants.kp, 0, 0, Constants.kf);

        // ── Intake & feeder ─────────────────────────────────────────────
        intake = new Motor(hardwareMap, "intake");
        feeder = new Motor(hardwareMap, "feeder");
        intake.setRunMode(Motor.RunMode.RawPower);
        feeder.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // ── Hood servo ──────────────────────────────────────────────────
        tiltServo = hardwareMap.get(Servo.class, "hood");
        tiltServo.setPosition(clampHood(HOOD_POSITION));

        telemetry.addData("Status", "Ready. GP2: Y=launcher  A=intake  RB=feeder");
        telemetry.addData("Max safe target", MAX_SAFE_VELOCITY + " ticks/sec (max " + MAX_TICKS_SEC + ")");
        telemetry.update();
    }

    // ════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {

        gamepadEx.readButtons();

        // ── Pinpoint localization ────────────────────────────────────────
        follower.update();
        Pose   robotPose = follower.getPose();
        double robotX    = robotPose.getX();
        double robotY    = robotPose.getY();

        // ── Distance to goal ─────────────────────────────────────────────
        double dx       = GOAL_X - robotX;
        double dy       = GOAL_Y - robotY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        // ── Which regression set? ─────────────────────────────────────────
        boolean closeSide = robotY > REGRESSION_SPLIT_Y;
        String regressionSide = closeSide ? "CLOSE (y > 48)" : "FAR   (y < 48)";

        // ── Drivetrain — delegate entirely to DrivetrainSubsystem ────────
        drivetrain.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);

        // ── Turret — keep target fresh, delegate update to TurretAiming ──
        turret.setTarget(new Pose(GOAL_X, GOAL_Y));
        turret.update();

        // ── Hood — Dashboard sets position ───────────────────────────────
        tiltServo.setPosition(clampHood(HOOD_POSITION));

        // ── Intake toggle (gamepad2 A) ───────────────────────────────────
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            intakeRunning = !intakeRunning;
        }
        intake.set(intakeRunning ? 1.0 : 0.0);

        // ── Feeder toggle (gamepad2 RB) ──────────────────────────────────
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            feederRunning = !feederRunning;
        }
        feeder.set(feederRunning ? 1.0 : -0.3);

        // ── Launcher toggle (gamepad2 Y) ─────────────────────────────────
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
            if (!running) {
                running = true;
                pidfRight.reset();
                pidfLeft.reset();
            } else {
                running = false;
                launcherRight.setPower(0);
                launcherLeft.setPower(0);
            }
        }

        double rightVelocity = launcherRight.getVelocity();
        double leftVelocity  = launcherLeft.getVelocity();
        double safeTarget    = Math.min(LAUNCHER_TARGET_VELOCITY, MAX_SAFE_VELOCITY);

        if (running) {
            double rightPower = Math.max(0, Math.min(1,
                    pidfRight.calculate(rightVelocity, safeTarget)));
            double leftPower  = Math.max(0, Math.min(1,
                    pidfLeft.calculate(leftVelocity,   safeTarget)));
            launcherRight.setPower(rightPower);
            launcherLeft.setPower(leftPower);
            telemetry.addData("rightPower", rightPower);
            telemetry.addData("leftPower",  leftPower);
        }

        // ── Telemetry ────────────────────────────────────────────────────
        telemetry.addLine("─── Record This Pair ───");
        telemetry.addData("Regression Set",  regressionSide);
        telemetry.addData("x₁ (distance)",  "%.2f in", distance);
        telemetry.addData("y₁ (position)",  "%.4f",    clampHood(HOOD_POSITION));
        telemetry.addLine("  Desmos: y₁ ~ ax₁² + bx₁ + c  (quadratic)");
        telemetry.addLine("  Collect 6-8 pts per side, run regression separately");
        telemetry.addLine("");
        telemetry.addData("Distance to Goal","%.2f in  (%.1f ft)", distance, distance / 12.0);
        telemetry.addData("Hood Position",   "%.4f", clampHood(HOOD_POSITION));
        telemetry.addData("Robot y",         "%.2f  →  %s", robotY, regressionSide);
        telemetry.addLine("");
        telemetry.addData("status",          running ? "RUNNING" : "STOPPED");
        telemetry.addData("targetVelocity",  safeTarget);
        telemetry.addData("rightVelocity",   rightVelocity);
        telemetry.addData("leftVelocity",    leftVelocity);
        telemetry.addData("rightError",      safeTarget - rightVelocity);
        telemetry.addData("leftError",       safeTarget - leftVelocity);
        telemetry.addData("atSpeed",         Math.abs(rightVelocity - safeTarget) < VELOCITY_TOLERANCE
                && Math.abs(leftVelocity  - safeTarget) < VELOCITY_TOLERANCE);
        telemetry.addData("kP/kF (Constants)", Constants.kp + " / " + Constants.kf);
        telemetry.addLine("");
        telemetry.addData("turret aimed",    turret.isAimed());
        telemetry.addData("turret error°",   "%.1f°", turret.getErrorDegrees());
        telemetry.addData("turret ticks",    "cur=%.0f  tgt=%.0f",
                turret.getCurrentTicks(), turret.getTargetTicks());
        telemetry.addLine("");
        telemetry.addData("intake",  intakeRunning ? "ON" : "OFF");
        telemetry.addData("feeder",  feederRunning ? "ON" : "OFF");
        telemetry.addLine("");
        telemetry.addData("x",       "%.2f in", robotX);
        telemetry.addData("y",       "%.2f in", robotY);
        telemetry.addData("heading", "%.1f°",   Math.toDegrees(robotPose.getHeading()));
        telemetry.update();
    }

    // ════════════════════════════════════════════════════════════════════
    @Override
    public void stop() {
        launcherRight.setPower(0);
        launcherLeft.setPower(0);
        intake.set(0);
        feeder.set(0);
        drivetrain.stop();
        turret.disable();
    }

    // ── Helpers ──────────────────────────────────────────────────────────
    private double clampHood(double pos) {
        return Math.max(MIN_POSITION, Math.min(MAX_POSITION, pos));
    }
}