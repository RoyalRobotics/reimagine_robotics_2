package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

public class GyroAutonomous extends LinearOpMode {
    DcMotor right;
    DcMotor left;

    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");

        GyroSensor gyro1;


        right.setDirection(DcMotor.Direction.FORWARD);
        left .setDirection(DcMotor.Direction.FORWARD);

        ModernRoboticsI2cGyro mrGyro;


    }
}
