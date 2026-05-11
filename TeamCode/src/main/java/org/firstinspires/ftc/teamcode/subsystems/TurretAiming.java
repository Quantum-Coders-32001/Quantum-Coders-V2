package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class TurretAiming {

    // ── Constants ─────────────────────────────────────────────────────────────
    public static final double TICKS_PER_DEGREE = 93.0;
    public static final double MIN_TICKS        = -13280;
    public static final double MAX_TICKS        = 22566;
    public static final double DEAD_ZONE        = 1000;
    public static final double MAX_POWER        = 1.0;

    // PIDF gains
    public static double KP = 0.02;
    public static double KI = 0.000;
    public static double KD = 0.001;
    public static double KF = 0.0;
    public static double KS = 0.2;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final CRServo   servoLeft, servoRight;
    private final DcMotorEx encoder;
    private final Follower  follower;

    // ── Control ───────────────────────────────────────────────────────────────
    private final PIDFController pidf;
    private Pose   target      = new Pose(0, 0);
    private double targetTicks = 0;
    private boolean active     = false;

    public TurretAiming(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        servoLeft  = hardwareMap.get(CRServo.class, "lturret");
        servoRight = hardwareMap.get(CRServo.class, "rturret");
        servoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        servoRight.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder = hardwareMap.get(DcMotorEx.class, "rf");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDFController(KP, KI, KD, KF);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    public void setTarget(Pose target) {
        this.target = target;
    }

    public void update() {
        pidf.setPIDF(KP, KI, KD, KF);

        if (!active) {
            servoLeft.setPower(0);
            servoRight.setPower(0);
            return;
        }

        targetTicks = calculateTargetTicks();

        double current = getCurrentTicks();
        double error   = targetTicks - current;

        double output = 0;
        if (Math.abs(error) >= DEAD_ZONE) {
            output = Range.clip(pidf.calculate(current, targetTicks), -MAX_POWER, MAX_POWER);
            if (KS > 0) {
                output = Range.clip(output + Math.copySign(KS, output), -MAX_POWER, MAX_POWER);
            }
        } else {
            pidf.reset();
        }

        servoLeft.setPower(output);
        servoRight.setPower(output);
    }

    public void enable() {
        active = true;
    }

    public void disable() {
        active = false;
        pidf.reset();
        servoLeft.setPower(0);
        servoRight.setPower(0);
    }

    public boolean isAimed() {
        return Math.abs(targetTicks - getCurrentTicks()) < DEAD_ZONE;
    }

    public double getCurrentTicks() {
        return encoder.getCurrentPosition();
    }

    public double getTargetTicks() {
        return targetTicks;
    }

    public double getErrorTicks() {
        return targetTicks - getCurrentTicks();
    }

    public double getErrorDegrees() {
        return getErrorTicks() / TICKS_PER_DEGREE;
    }

    // ── Internal math ─────────────────────────────────────────────────────────

    private double calculateTargetTicks() {
        Pose robotPose = follower.getPose();

        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();

        double angleToGoalField = Math.toDegrees(Math.atan2(dy, dx));
        double angleToGoalRobot = angleToGoalField - Math.toDegrees(robotPose.getHeading());

        double routedAngle = routeTurret(angleToGoalRobot);

        return routedAngle * TICKS_PER_DEGREE;
    }

    private double routeTurret(double degrees) {
        double minDeg = MIN_TICKS / TICKS_PER_DEGREE;
        double maxDeg = MAX_TICKS / TICKS_PER_DEGREE;

        if (degrees >= minDeg && degrees <= maxDeg) return degrees;

        for (int i = -5; i <= 5; i++) {
            double candidate = degrees + (i * 360);
            if (candidate >= minDeg && candidate <= maxDeg) return candidate;
        }

        return Range.clip(degrees, minDeg, maxDeg);
    }
}