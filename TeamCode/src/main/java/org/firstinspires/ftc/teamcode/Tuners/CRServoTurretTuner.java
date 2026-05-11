package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config("CRServoTurretTuner")
@TeleOp(name = "sigma bot", group = "tuning")
public class CRServoTurretTuner extends OpMode {

    private static final String SERVO_LEFT_NAME  = "lturret";
    private static final String SERVO_RIGHT_NAME = "rturret";
    private static final String ENCODER_NAME     = "rf";

    public static double TARGET_POSITION = 0;

    public static double KP        = 0.0;
    public static double KI        = 0.0;
    public static double KD        = 0.0;
    public static double KF        = 0.0;
    public static double KS        = 0.0;

    public static double MAX_POWER = 1.0;
    public static double DEAD_ZONE = 5;

    public static boolean REVERSE_LEFT  = false;
    public static boolean REVERSE_RIGHT = true;

    private CRServo        servoLeft, servoRight;
    private DcMotorEx      encoder;
    private PIDFController pidf;
    private Telemetry      dash;

    private long lastTimeMs = 0;

    @Override
    public void init() {
        dash = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        servoLeft  = hardwareMap.get(CRServo.class, SERVO_LEFT_NAME);
        servoRight = hardwareMap.get(CRServo.class, SERVO_RIGHT_NAME);
        servoLeft.setDirection(REVERSE_LEFT   ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servoRight.setDirection(REVERSE_RIGHT ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDFController(KP, KI, KD, KF);

        lastTimeMs = System.currentTimeMillis();

        dash.addLine("Ready. Set TARGET_POSITION in dashboard to move.");
        dash.update();
    }

    @Override
    public void loop() {
        pidf.setPIDF(KP, KI, KD, KF);

        double current = encoder.getCurrentPosition();
        double error   = TARGET_POSITION - current;

        double output = 0;
        if (Math.abs(error) >= DEAD_ZONE) {
            output = clamp(pidf.calculate(current, TARGET_POSITION), -MAX_POWER, MAX_POWER);

            // S — static friction kick
            if (KS > 0) {
                output += Math.copySign(KS, output);
                output  = clamp(output, -MAX_POWER, MAX_POWER);
            }
        } else {
            pidf.reset();
        }

        servoLeft.setPower(output);
        servoRight.setPower(output);

        dash.addData("Target",  "%.1f ticks", TARGET_POSITION);
        dash.addData("Current", "%.1f ticks", current);
        dash.addData("Error",   "%.2f ticks", error);
        dash.addData("Output",  "%.5f",       output);
        dash.addData("Loop dt", "%.4f s",     (System.currentTimeMillis() - lastTimeMs) / 1000.0);
        dash.update();

        lastTimeMs = System.currentTimeMillis();
    }

    @Override
    public void stop() {
        servoLeft.setPower(0);
        servoRight.setPower(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}