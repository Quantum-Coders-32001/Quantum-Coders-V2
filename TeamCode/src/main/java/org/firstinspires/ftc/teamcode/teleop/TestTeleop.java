package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.TurretAiming;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.tuners.Constants;

@TeleOp(name = "TELEOP TEST", group = "TeleOp")
public class TestTeleop extends OpMode {

    private static final double kP = 0.005;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.00075;

    private static final double TARGET_VELOCITY = 1800;
    private static final double STEP            = 50;
    private static final double MAX_SAFE_TARGET = 2300;

    private Motor      intake, feeder;
    private GamepadEx  gamepadEx;
    private boolean    intakeRunning = false;

    private DcMotorEx      launcherRight, launcherLeft;
    private PIDFController pidfRight, pidfLeft;

    private boolean running = false;
    private boolean lastY   = false, lastB = false, lastLB = false, lastLT = false;

    private double targetVelocity = TARGET_VELOCITY;

    private DrivetrainSubsystem drivetrain;
    private Follower             follower;
    private TurretAiming         turretAiming;

    private static final Pose GOAL_POSE = new Pose(132, 139);

    @Override
    public void init() {
        gamepadEx = new GamepadEx(gamepad1);

        // Drivetrain
        drivetrain = new DrivetrainSubsystem(hardwareMap);

        // Pedro follower for localization only
        follower = Constants.createFollower(hardwareMap);

        // Turret — always enabled
        turretAiming = new TurretAiming(hardwareMap, follower);
        turretAiming.setTarget(GOAL_POSE);
        turretAiming.enable();

        // Launcher
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
        launcherRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidfRight = new PIDFController(kP, kI, kD, kF);
        pidfLeft  = new PIDFController(kP, kI, kD, kF);

        telemetry.addData("Status", "Ready. Y=launcher | B=stop | LB/LT=velocity | A=intake");
        telemetry.update();
    }

    @Override
    public void loop() {
        gamepadEx.readButtons();

        // ── Drive + Localization ──────────────────────────────────────────────
        follower.update();
        drivetrain.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // ── Turret ────────────────────────────────────────────────────────────
        turretAiming.update();

        // ── Intake ────────────────────────────────────────────────────────────
        intake.set(1);
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            intakeRunning = !intakeRunning;
            feeder.set(intakeRunning ? 1 : 0);
        }

        // ── Launcher ──────────────────────────────────────────────────────────
        boolean y  = gamepad1.y;
        boolean b  = gamepad1.b;
        boolean lb = gamepad1.left_bumper;
        boolean lt = gamepad1.left_trigger > 0.1;

        if (y && !lastY) { running = true;  pidfRight.reset(); pidfLeft.reset(); }
        if (b && !lastB) { running = false; launcherRight.setPower(0); launcherLeft.setPower(0); }
        if (lb && !lastLB) targetVelocity = Math.min(targetVelocity + STEP, MAX_SAFE_TARGET);
        if (lt && !lastLT) targetVelocity = Math.max(targetVelocity - STEP, 0);
        lastY = y; lastB = b; lastLB = lb; lastLT = lt;

        double rightVelocity = launcherRight.getVelocity();
        double leftVelocity  = launcherLeft.getVelocity();

        if (running) {
            launcherRight.setPower(Math.max(0, Math.min(1, pidfRight.calculate(rightVelocity, targetVelocity))));
            launcherLeft.setPower(Math.max(0, Math.min(1, pidfLeft.calculate(leftVelocity, targetVelocity))));
            telemetry.addData("rightPower", launcherRight.getPower());
            telemetry.addData("leftPower",  launcherLeft.getPower());
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("intake",         intakeRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("launcher",       running ? "RUNNING" : "STOPPED");
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("rightVelocity",  rightVelocity);
        telemetry.addData("leftVelocity",   leftVelocity);
        telemetry.addData("turretTicks",    turretAiming.getCurrentTicks());
        telemetry.addData("turretTarget",   turretAiming.getTargetTicks());
        telemetry.addData("turretError",    turretAiming.getErrorDegrees() + "°");
        telemetry.addData("turretAimed",    turretAiming.isAimed());
        telemetry.addData("robotPose",      follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.set(0);
        feeder.set(0);
        launcherRight.setPower(0);
        launcherLeft.setPower(0);
        turretAiming.disable();
    }
}