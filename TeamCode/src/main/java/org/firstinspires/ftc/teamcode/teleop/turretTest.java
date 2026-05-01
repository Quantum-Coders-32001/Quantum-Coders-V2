package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "CR Servo Spin", group = "Test")
public class turretTest extends LinearOpMode {

    CRServo servo1;
    CRServo servo2;

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPower(1.0);
            servo2.setPower(1.0);
        }
    }
}