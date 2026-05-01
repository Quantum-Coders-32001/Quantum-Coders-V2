package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LAUNCHER ONLY", group = "Teleop")
public class shooterTest extends OpMode {

    final double LAUNCHER_TARGET_VELOCITY = 1200;
    final double LAUNCHER_MAX_VELOCITY    = 6000;
    final double LAUNCHER_MIN_ADJUST      = 0;
    final double LAUNCHER_STEP            = 50;
    final double VELOCITY_TOLERANCE       = 20;

    final double SERVO_SPEED    = 0.003;  // position units per loop — lower = slower/less sensitive
    final double SERVO_DEADBAND = 0.08;
    final double SERVO_MIN      = 0.0;    // 0°
    final double SERVO_MAX      = 1.0;    // 300° (GoBilda standard)
    final double SERVO_START    = 0.5;    // start at 150°

    private DcMotorEx launcherRight, launcherLeft;
    private Servo tiltServo;
    private DcMotor intake, feeder;

    private enum LaunchState { IDLE, SPIN_UP, LAUNCH }
    private LaunchState launchState = LaunchState.IDLE;

    private double launcherTargetVelocity = 0;
    private boolean lastLB = false, lastLT = false;
    private double servoPosition = SERVO_START;

    @Override
    public void init() {
        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launch1");
        tiltServo     = hardwareMap.get(Servo.class, "tilt");

        launcherRight.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);

        launcherRight.setZeroPowerBehavior(BRAKE);
        launcherLeft.setZeroPowerBehavior(BRAKE);

        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tiltServo.setPosition(servoPosition);

        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // ===== TILT SERVO (gamepad1 left stick Y, low sensitivity) =====
        double stick = -gamepad1.left_stick_y; // positive = up
        if (Math.abs(stick) > SERVO_DEADBAND) {
            servoPosition += stick * SERVO_SPEED;
            servoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPosition));
            tiltServo.setPosition(servoPosition);
        }

        // ===== LAUNCHER STATE MACHINE =====
        switch (launchState) {
            case IDLE:
                setLauncherVelocity(0);
                if (gamepad1.y) {
                    launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY;
                    setLauncherVelocity(launcherTargetVelocity);
                    launchState = LaunchState.SPIN_UP;
                } else if (gamepad1.x) {
                    launcherTargetVelocity = 1500;
                    setLauncherVelocity(launcherTargetVelocity);
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                setLauncherVelocity(launcherTargetVelocity);
                if (Math.abs(launcherRight.getVelocity() - launcherTargetVelocity) < VELOCITY_TOLERANCE) {
                    launchState = LaunchState.LAUNCH;
                }
                if (gamepad1.b && !gamepad1.y && !gamepad1.x) {
                    setLauncherVelocity(0);
                    launchState = LaunchState.IDLE;
                }
                break;

            case LAUNCH:
                setLauncherVelocity(launcherTargetVelocity);
                if (gamepad1.b && !gamepad1.y && !gamepad1.x) {
                    launchState = LaunchState.IDLE;
                }
                break;
        }

        // ===== RPM ADJUST (LB = up, LT = down) =====
        boolean lb = gamepad1.left_bumper;
        boolean lt = gamepad1.left_trigger > 0.1;
        if (lb && !lastLB) {
            launcherTargetVelocity = Math.min(launcherTargetVelocity + LAUNCHER_STEP, LAUNCHER_MAX_VELOCITY);
            if (launchState != LaunchState.IDLE) setLauncherVelocity(launcherTargetVelocity);
        }
        if (lt && !lastLT) {
            launcherTargetVelocity = Math.max(launcherTargetVelocity - LAUNCHER_STEP, LAUNCHER_MIN_ADJUST);
            if (launchState != LaunchState.IDLE) setLauncherVelocity(launcherTargetVelocity);
        }
        lastLB = lb;
        lastLT = lt;

        intake.setPower(1);

        // ===== INTAKE / FEEDER CONTROLS =====
        if (gamepad1.a) {
            feeder.setPower(1);
        } else {
            feeder.setPower(-0.3);
        }

        // ===== TELEMETRY =====
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Target Velocity", launcherTargetVelocity);
        telemetry.addData("Actual Velocity R", launcherRight.getVelocity());
        telemetry.addData("Actual Velocity L", launcherLeft.getVelocity());
        telemetry.addData("At Speed", Math.abs(launcherRight.getVelocity() - launcherTargetVelocity) < VELOCITY_TOLERANCE);
        telemetry.addData("Servo Position", String.format("%.3f  (%.1f°)", servoPosition, servoPosition * 300));
        telemetry.addData("Intake/Feeder", gamepad1.a ? "ON" : "OFF");
        telemetry.update();
    }

    private void setLauncherVelocity(double v) {
        launcherRight.setVelocity(v);
        launcherLeft.setVelocity(v);
    }

    @Override
    public void stop() {
        setLauncherVelocity(0);
        intake.setPower(0);
        feeder.setPower(0);
    }
}