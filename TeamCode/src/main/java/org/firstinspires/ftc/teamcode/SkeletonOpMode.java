package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

@TeleOp(name="Basicc",group=" ")
public class SkeletonOpMode extends LinearOpMode {
    /* 1: INITIALIZATION
     Initialize all variables here */

    robot robo = new robot();

    @Override
    public void runOpMode() throws InterruptedException {
    /* 2: CONFIGURATION
    Connect all variables to the robot (not Boolean, Integer, etc but Motors, Sensors)
     */

        robo.init(hardwareMap, telemetry);

        waitForStart();

    /* 3: EXECUTION
    Actual programing loop, write code here
    Add while() loop if teleop, don't if autonomous
     */

        while (opModeIsActive()) {

        robo.Right.setPower(-gamepad1.right_stick_y);
        robo.Left.setPower(-gamepad1.left_stick_y);

        }
    }
}