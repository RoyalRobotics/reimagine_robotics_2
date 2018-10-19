package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonomousPrototype1 extends LinearOpMode {


    Servo servoA;
    boolean toggleA = false;
    double SERVOSTATE;

    @Override
    public void runOpMode() {

        servoA = hardwareMap.get(Servo.class, "A");

        waitForStart();

        while(opModeIsActive()) {

            SERVOSTATE = (toggleA ? 1.0 : 0.0);
            servoA.setPosition(SERVOSTATE);

            if (gamepad1.a && toggleA){
                toggleA = false;
            }
            if (gamepad1.a && !toggleA){
                toggleA =true;
            }



            }
        }
    }
}
