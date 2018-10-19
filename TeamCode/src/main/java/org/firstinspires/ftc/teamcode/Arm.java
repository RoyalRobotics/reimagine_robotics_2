package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends LinearOpMode {

    double linka, linkb, linkc = 19; // cm
    Servo servoB, servoC, servoD;
    DcMotorEx leftMotor, rightMotor, bevelMotor, motorA;

    Double x, y = 0.0;


    @Override
    public void runOpMode() {

        servoB = hardwareMap.get(Servo.class, "B");
        servoC = hardwareMap.get(Servo.class, "C");
        servoD = hardwareMap.get(Servo.class, "D");

        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right");
        bevelMotor = hardwareMap.get(DcMotorEx.class, "bevel");
        motorA = hardwareMap.get(DcMotorEx.class, "arm");

        telemetry.addLine("Ready to go.");
        telemetry.update();

        waitForStart();
        
        while(opModeIsActive(){








            telemetry.addData("SERVO B", servoB.getPosition());
            telemetry.addData("SERVO C", servoC.getPosition());
            telemetry.addData("SERVO D", servoD.getPosition());
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
        }


    }
}
