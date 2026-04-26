package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DtTest", group = "StarterBot")
public class drivetrainTest extends OpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor intake = null;
    private DcMotor feeder = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        feeder.setZeroPowerBehavior(BRAKE);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.y) {
            intake.setPower(1);
            feeder.setPower(1);
        } else {
            intake.setPower(0);
            feeder.setPower(0);
        }
    }

    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}