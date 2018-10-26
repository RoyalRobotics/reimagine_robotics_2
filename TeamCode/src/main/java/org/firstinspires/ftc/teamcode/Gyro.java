package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Gyro extends LinearOpMode {

    double intital, error, integral, deriv, errorTotal, interval, r, z = 0;
    double[] accum;
    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx rightMotor, leftMotor, bevelMotor, armMotor;
    Servo servoB, servoC, servoD;

    BNO055IMU imu;


    @Override
    public void runOpMode() {

        servoB = hardwareMap.get(Servo.class, "B");
        servoC = hardwareMap.get(Servo.class, "C");
        servoD = hardwareMap.get(Servo.class, "D");

        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right");
        bevelMotor = hardwareMap.get(DcMotorEx.class, "bevel");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData(".getAngularOrientation", imu.getAngularOrientation());
            telemetry.addData(".getAngularVelocity",imu.getAngularVelocity());
            telemetry.addData(".getGravity",imu.getGravity());
            telemetry.addData(".getParameters",imu.getParameters());
            telemetry.addData(".getVelocity", imu.getVelocity());
        }


    }


    public void rotateDegrees(double degrees){


    }



    public double getError(){
        error = r - z;
        return error;
    }

    public double calculateIntegral(){
        double total = 0;
        for(int i = 0; i < accum.length; i++){
            if(i != 0 && i != accum.length){
                total = total + 2*accum[i];
            }else{
                total = total + accum[i];
            }
        }
        integral = 0.5 *(1/1000)*(total);
        return  integral;
    }

    public double calculateDerivitive(){
        deriv = (error-intital)/(runtime.milliseconds());
        return  deriv;
    }


}
