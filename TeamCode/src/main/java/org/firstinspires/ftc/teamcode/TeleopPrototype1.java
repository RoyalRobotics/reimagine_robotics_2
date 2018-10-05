package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TP1", group = "")
public class TeleopPrototype1 extends LinearOpMode{
    public DcMotor right;
    public DcMotor left;
    //public DcMotor arm;

    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(DcMotor.class, "R");
        left = hardwareMap.get(DcMotor.class, "L");
        //arm = hardwareMap.get(DcMotor.class, "A");


        right.setDirection(DcMotorSimple.Direction.REVERSE);
        //left.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("bro r u good");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("finish them");
            telemetry.update();

            right.setPower(-gamepad1.right_stick_y);
            left.setPower(-gamepad1.left_stick_y);
            // arm.setPower(-gamepad2.right_stick_y);


        }
    }


    }

