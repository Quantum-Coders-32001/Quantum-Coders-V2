package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * LAUNCHER PIDF TUNING OPMODE — FTC Dashboard
 *
 * Hardware:
 *   - 2x GoBILDA 6000 RPM motors (direct drive)
 *   - 2x 96mm Hogback Traction Wheels
 *   - 2x SWYFT 72mm Steel Flywheels (186g, 480 g·cm² each)
 *
 * SETUP:
 *   1. Add to build.dependencies.gradle:
 *        implementation 'com.acmerobotics.dashboard:dashboard:0.4.16'
 *   2. Connect to robot Wi-Fi
 *   3. Open <a href="http://192.168.43.1:8080/dash">...</a> in your browser
 *   4. Run this OpMode
 *   5. Edit kP, kI, kD, kF, TARGET_VELOCITY in the "Variable Configuration" panel (right side)
 *   6. Click the graph icon next to rightVelocity / leftVelocity / targetVelocity to plot them
 *
 * TUNING STEPS:
 *   1. kP=0, kI=0, kD=0, kF=13 → Press Y → slowly raise kF until motor reaches ~90% of target
 *   2. Raise kP (20→40→60→80) until it snaps to target quickly without bouncing
 *   3. Fire balls, watch graph — good tune = small dip, fast recovery
 *
 * CONTROLS (gamepad1):
 *   Y   — Spin up   |   B  — Stop
 *   LB  — Target +STEP  |   LT — Target -STEP
 */

@Config
@TeleOp(name = "Launcher PIDF Tuner", group = "Tuning")
public class LauncherTuningSir extends OpMode {

    // These appear in the FTC Dashboard "Variable Configuration" panel — edit live!
    public static double kP = 80;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 14;

    public static double TARGET_VELOCITY = 1200; // ticks/sec
    public static double STEP = 50;

    private DcMotorEx launcherRight, launcherLeft;

    private boolean running = false;
    private boolean lastY = false, lastB = false, lastLB = false, lastLT = false;

    @Override
    public void init() {
        // Hook into both Driver Hub and FTC Dashboard simultaneously
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launch1");

        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDF();

        telemetry.addData("Status", "Ready. Y = spin up  |  B = stop  |  LB/LT = adjust target");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Re-apply every loop so dashboard variable changes take effect immediately
        applyPIDF();

        boolean y  = gamepad1.y;
        boolean b  = gamepad1.b;
        boolean lb = gamepad1.left_bumper;
        boolean lt = gamepad1.left_trigger > 0.1;

        if (y && !lastY)  running = true;
        if (b && !lastB) {
            running = false;
            launcherRight.setVelocity(0);
            launcherLeft.setVelocity(0);
        }
        if (lb && !lastLB) TARGET_VELOCITY = Math.min(TARGET_VELOCITY + STEP, 6000);
        if (lt && !lastLT) TARGET_VELOCITY = Math.max(TARGET_VELOCITY - STEP, 0);

        lastY = y; lastB = b; lastLB = lb; lastLT = lt;

        if (running) {
            launcherRight.setVelocity(TARGET_VELOCITY);
            launcherLeft.setVelocity(TARGET_VELOCITY);
        }

        double rightVelocity = launcherRight.getVelocity();
        double leftVelocity  = launcherLeft.getVelocity();

        // These all show up as graphable in the dashboard
        telemetry.addData("status",         running ? "RUNNING" : "STOPPED");
        telemetry.addData("targetVelocity", TARGET_VELOCITY);  // <-- graph this
        telemetry.addData("rightVelocity",  rightVelocity);    // <-- graph this
        telemetry.addData("leftVelocity",   leftVelocity);     // <-- graph this
        telemetry.addData("rightError",     TARGET_VELOCITY - rightVelocity);
        telemetry.addData("leftError",      TARGET_VELOCITY - leftVelocity);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcherRight.setVelocity(0);
        launcherLeft.setVelocity(0);
    }

    private void applyPIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(kP, kI, kD, kF);
        launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }
}