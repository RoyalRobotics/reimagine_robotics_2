package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class TEST extends LinearOpMode {

    public  DcMotor Left;
    public  DcMotor Right;
    public  DcMotor Arm;

    public  GyroSensor gyro;
    public  ModernRoboticsI2cGyro mrGyro;

    int zV;

    // VuforiaLocalizer vuforia;

    // gyro_source gyrocodec = new gyro_source();



    @Override
    public void runOpMode() throws InterruptedException{

        Right = hardwareMap.get(DcMotor.class, "R");
        Left = hardwareMap.get(DcMotor.class, "L");

        // Arm = hardwareMap.get(DcMotor.class, "A");


        gyro = hardwareMap.gyroSensor.get("G");
        mrGyro = (ModernRoboticsI2cGyro) gyro;

        //mrGyro.calibrate();

        /*while (mrGyro.isCalibrating()){
            telemetry.addLine("Calibrating...");
            telemetry.update();
        }
*/

        Left.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Gyro has been calibrated");
        telemetry.addLine("Ready to go compadre");
        telemetry.update();

        waitForStart();

        telemetry.addLine("good");
        Thread.sleep(1000);


    }
}
