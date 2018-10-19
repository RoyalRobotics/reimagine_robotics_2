package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TP2", group = "")
public class TeleopPrototype1 extends LinearOpMode{
    public DcMotor right;
    public DcMotor left;
    public DcMotor arm;
    public DcMotor a2;
    public Servo s1;
    public Servo s2;
    public Servo s3;

    //public DcMotor arm;

    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(DcMotor.class, "R");
        left = hardwareMap.get(DcMotor.class, "L");
        //arm = hardwareMap.get(DcMotor.class, "A");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        //left.setDirection(DcMotorSimple.Direction.FORWARD);
        s1.setPosition(0.0);
        s2.setPosition(0.0);
        s3.setPosition(0.0);
        telemetry.addLine("bro r u good");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("finish them");
            telemetry.update();

            right.setPower(-gamepad1.right_stick_y);
            left.setPower(-gamepad1.left_stick_y);
            arm.setPower(-gamepad2.right_stick_y);
            if (gamepad2.a)
            {
                s1.setPosition(.5);
            }
            else {
                s1.setPosition(0);
            }

            if (gamepad2.b)
            {
                s2.setPosition(.5);
            }
            else {
                s2.setPosition(0);
            }
            if (gamepad2.x)
            {
                s3.setPosition(.5);
            }
            else
            {
                s3.setPosition(0);
            }

        }
    }
    }




