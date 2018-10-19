package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

public class Arm extends LinearOpMode {


    ElapsedTime runner = new ElapsedTime();

    double linka, linkb, linkc = 19; // cm
    Servo servoB, servoC, servoD;
    DcMotorEx leftMotor, rightMotor, bevelMotor, motorA;

    Double x, y, a, b, c = 0.0;
    Double lastTimeY = 0.0;
    Double lastTimeX = 0.0;

    final static double SERVO_B_POS_0 = 0;
    final static double SERVO_C_POS_0 = 0;
    final static double SERVO_D_POS_0 = 0;

    final static double SERVO_B_POS_90 = 0;
    final static double SERVO_C_POS_90= 0;
    final static double SERVO_D_POS_90 = 0;

    final static double SERVO_B_CONVERSION_FACTOR = (SERVO_B_POS_90-SERVO_B_POS_0)/90;
    final static double SERVO_C_CONVERSION_FACTOR = (SERVO_C_POS_90-SERVO_C_POS_0)/90;
    final static double SERVO_D_CONVERSION_FACTOR = (SERVO_D_POS_90-SERVO_D_POS_0)/90;

    @Override
    public void runOpMode() {

        servoB = hardwareMap.get(Servo.class, "B");
        servoC = hardwareMap.get(Servo.class, "C");
        servoD = hardwareMap.get(Servo.class, "D");

        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right");
        bevelMotor = hardwareMap.get(DcMotorEx.class, "bevel");
        motorA = hardwareMap.get(DcMotorEx.class, "arm");

        servoB.setPosition(SERVO_B_POS_0);
        servoC.setPosition(SERVO_C_POS_0);
        servoD.setPosition(SERVO_D_POS_0);

        telemetry.addLine("Ready to go.");
        telemetry.update();

        waitForStart();


        while(opModeIsActive()){

            double phi = servoD.getPosition();

            if(gamepad1.left_stick_y != 0 && (lastTimeX - runner.milliseconds()) > 300){
                x = x -gamepad1.left_stick_y * 0.01;
                lastTimeX = runner.milliseconds();
            }

            if(gamepad1.left_stick_x != 0 && (lastTimeX - runner.milliseconds()) > 300){
                x = x -gamepad1.left_stick_x * 0.01;
                lastTimeX = runner.milliseconds();
            }

            if(gamepad1.right_trigger > 0.1){
                servoD.setPosition((servoD.getPosition() < 1.0 ? servoD.getPosition() + gamepad1.right_trigger/20 : servoD.getPosition()));
            }

            if(gamepad1.left_trigger > 0.1){
                servoD.setPosition((servoD.getPosition() < 0.0 ? servoD.getPosition() - gamepad1.right_trigger/20 : servoD.getPosition()));
            }

            if(gamepad1.right_bumper){

            }

            LinkedList<Double> values = invKinematics(x,y,phi);

            a = values.get(0);
            b = values.get(1);
            c = values.get(2);


            servoB.setPosition(a);
            servoC.setPosition(b);
            servoD.setPosition(c);


            telemetry.addData("SERVO B", servoB.getPosition());
            telemetry.addData("SERVO C", servoC.getPosition());
            telemetry.addData("SERVO D", servoD.getPosition());
            telemetry.addData("X", x);
            telemetry.addData("Y", y);

        }







    }
    public LinkedList<Double> invKinematics(double x, double y, double phi){
            LinkedList<Double> angles = new LinkedList<Double>(); //initializes list that outputs will be added to
            //joint angles will folow q1, q2, ... pattern

            double L1 = linka; //link lengths need to be set correctly
            double L2 = linkb; //units probably should be inches
            double L3 = linkc;

            double Xe = x; //sets x value of the end effector
            double Ye = y; // sets z value of the end effector

            double Xw = Xe - L3*Math.cos(Math.toRadians(phi)); //finds x-z value of the wrist
            double Yw = Ye - L3*Math.sin(Math.toRadians(phi));

            double alpha = Math.atan2(Yw, Xw);
            double gamma = Math.acos(((Xw * Xw) + (Yw * Yw) + (L1 * L1) - (L2 * L2)) / (2 * L1 * Math.sqrt((Xw * Xw) + (Yw * Yw))));
            double q1 = alpha + gamma; //calculates first joint of the 3dof planar arm in the upright position

            //calculates middle join of planar 3dof arm
            double q2 = -1*(Math.PI - Math.acos(((L1*L1) + (L2*L2) - (Xw * Xw) - (Yw * Yw))/(2*L1*L2)));

            //calculates wrist joint of planar 3dof arm
            double q3 = phi - q1 - q2;

            angles.add(q1);
            angles.add(q2);
            angles.add(q3);

            return angles; //angles outputs in radians
    }
}