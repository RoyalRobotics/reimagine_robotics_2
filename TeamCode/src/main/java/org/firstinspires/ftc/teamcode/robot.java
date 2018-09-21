package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Thread.sleep;

public class robot {

    public static DcMotor Left;
    public static DcMotor Right;
    public static DcMotor Arm;

    public static GyroSensor gyro;
    public static ModernRoboticsI2cGyro mrGyro;

    int zV;

    VuforiaLocalizer vuforia;

    gyro_source gyrocodec = new gyro_source();


    public void init(HardwareMap hwm, Telemetry tel) throws InterruptedException {

        //Initialization for RelicRecoveryAutonomous.java

        Right = hwm.get(DcMotor.class, "R");
        Left = hwm.get(DcMotor.class, "L");

       // Arm = hwm.get(DcMotor.class, "A");


        gyro = hwm.gyroSensor.get("G");
        mrGyro = (ModernRoboticsI2cGyro) gyro;

        //mrGyro.calibrate();

        /*while (mrGyro.isCalibrating()){
            tel.addLine("Calibrating...");
            tel.update();
        }
*/

        Left.setDirection(DcMotor.Direction.REVERSE);

        tel.addLine("Gyro has been calibrated");
        tel.addLine("Ready to go compadre");
        tel.update();
    }


    @Deprecated
    public void basic_gyroTurn(double power, int angle) throws InterruptedException {

        //Turn right with gyro

        mrGyro.calibrate();
        Thread.sleep(1000);

        zV = mrGyro.getIntegratedZValue();

        Right.setPower(-power);
        Left.setPower(power);


        while (zV != -angle) {

        }

        Right.setPower(0);
        Left.setPower(0);

    }


    @Deprecated
    public void basic_leftGyroTurn(double power, int angle) throws InterruptedException {

        //turn left with gyro

        mrGyro.calibrate();
        Thread.sleep(1000);

        zV = mrGyro.getIntegratedZValue();

        Right.setPower(power);
        Left.setPower(-power);

        while (zV != angle) {

        }

        Right.setPower(0);
        Left.setPower(0);

    }


    public void gyroTelemetry(Telemetry tel) {

        //basic gyro telemetry

        zV = mrGyro.getIntegratedZValue();

        tel.addData(">", zV);
        tel.update();

    }



    public void left(double power, long millis) throws InterruptedException {

        //Basic move left

        Left.setPower(-power);
        Right.setPower(power);

        sleep(millis);

        Right.setPower(0);
        Left.setPower(0);


    }


    public void left(double power) {

        //Basic move left

        Right.setPower(power);
        Left.setPower(-power);

    }


    public void right(double power, long millis) throws InterruptedException {

        //Basic move right

        Right.setPower(-power);
        Left.setPower(power);

        sleep(millis);

        Right.setPower(0);
        Left.setPower(0);

    }


    public void right(double power){

        //Basic move right

        Right.setPower(-power);
        Left.setPower(power);
    }


    public void move(double power, long millis) throws InterruptedException {

        //Basic move forwards

        Right.setPower(power);
        Left.setPower(power);

        sleep(millis);

        Right.setPower(0);
        Left.setPower(0);

    }


    public void move(double power) {

        //Basic move forwards

        Right.setPower(power);
        Left.setPower(power);

    }


    public void stop(){
        move(0);  // lazy coding, amirite?
    }



    public void straighten(Telemetry t, LinearOpMode thi){

        if(mrGyro.getIntegratedZValue() > 0){

            gyrocodec.gyroAbsoluteTurn(0,-1,thi.opModeIsActive(),t);

        }else if(mrGyro.getIntegratedZValue() < 0){

            gyrocodec.gyroAbsoluteTurn(0,1,thi.opModeIsActive(),t);

        }
        else{



        }

    }

    public void vuforia(HardwareMap hwm, Telemetry tel) throws InterruptedException{

        /**
         * Make a decision based on vuforia decoding
         * For a better understanding @see TestVu
         **/

        int cameraMonitorViewId = hwm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwm.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AaoqgOL/////AAAAGbOH/YIsFkN4qAHOJSZVLf8gODe8T4TOfcAa/PBKY/8Py7aUNG/Hf2wvZT4OeCPqO+q4RYULQ1VjmxrsvwKtUPLpwH7InEZKH5MA9gD/X4j2Bz0O1say2B5okUBajZDZ6dnAY8q9ngcJNVKnFQqIBLlLIdRsy6S6JonETSJXNtVJpVLmL9A70AxEp4+0NwfAVH7rP5oTeckggK5lG/eRUPYVlOthkVCXTDEJCXB3vnGfbzy2hnUxwZtJkES3Hnk0w6RGJKazKOas1pM24dCiNHj2/Wtz3DrTK5IxHuICKplblKil2ecH6dV0+pDO8wCEjTJBAunIbLugU9ctKDQFaTOLL8Rdb9oJIzy/bhWyLM5s";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTracktables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTracktables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        sleep(1000);

        relicTracktables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if(vuMark == RelicRecoveryVuMark.RIGHT){

            right(1, 500);

            tel.addData("VuMark", "Right visible", vuMark);

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

            if(pose != null){

                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);

                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

            }

        }else if(vuMark == RelicRecoveryVuMark.LEFT){

            left(1, 500);

            tel.addData("VuMark", "Left visible", vuMark);

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

            if(pose != null){

                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);

                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

            }

        }else if(vuMark == RelicRecoveryVuMark.CENTER){

            move(-1, 500);

            tel.addData("VuMark", "Center visible", vuMark);

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

            if(pose != null){

                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);

                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

            }else{

                tel.addData("Vumark", "not visible");

            }

            tel.update();
        }
    }

    public void returnGeneralTelemetry(Telemetry telem){

        telem.addData("Gyro Z-Value", mrGyro.getIntegratedZValue());
        telem.addData("Gyro Heading", mrGyro.getHeading());
        telem.update();
    }
}
