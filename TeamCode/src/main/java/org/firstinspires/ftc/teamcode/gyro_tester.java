package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "Gyro Test", group = "")
public class gyro_tester extends LinearOpMode {

    robot robo = new robot();
    gyro_source gyro = new gyro_source();


    @Override
    public void runOpMode() throws InterruptedException {

        robo.init(hardwareMap, telemetry);

        waitForStart();

        robo.move(1, 100);
        Thread.sleep(1000);
        robo.right(1, 100);
        Thread.sleep(1000);
        gyro.gyroAbsoluteTurn(30, 1, opModeIsActive(), telemetry);
        Thread.sleep(1000);
        gyro.gyroRelativeTurn(30, 1, false, opModeIsActive(), telemetry );

        stop();


    }
}
