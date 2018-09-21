package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Thanzim on 10/8/2017.
 */

public class gyro_source extends robot{


    int heading;
    boolean CCW = false; // Stands for CounterClockwise
    int error = 1; // arbitrary value
    double pwfactor; //Power Factor

    public void gyroAbsoluteTurn(int angle, double factor, boolean state, Telemetry t) {

        heading = mrGyro.getHeading();
        error = (angle - heading);

        if(0 < error && error < 180){
            CCW = true;
        }
        else{
            CCW = false;

        }

        while(error != 0 && state) {

            returnGeneralTelemetry(t);

            if(angle > mrGyro.getHeading() && angle > 0) {
                error = angle - mrGyro.getHeading();
            }else{
                error = mrGyro.getHeading() - angle;
            }
            pwfactor = Range.clip(factor * Math.pow(1.2,(error - 90))+.2,-1,1);


            if(CCW){
                left(pwfactor);
            }
            else{
                right(pwfactor);
            }


            t.addData("pwf",pwfactor);
            t.addData("error",error);
            t.update();
        }
        t.addData("pwf",pwfactor);
        t.addData("error",error);
        t.update();
        stop();
    }

    public void gyroAbsoluteTurn(int angle, double factor, boolean ccw, boolean state, Telemetry t) {

        heading = mrGyro.getHeading();
        error = (angle - heading);
        CCW = ccw;

        while(error != 0 && state) {

            returnGeneralTelemetry(t);

            error = Math.abs(angle - mrGyro.getHeading());
            pwfactor = (factor * (error/90)+.25);

            if(!CCW){
                left(pwfactor);
            }
            else{
                right(pwfactor);
            }

        }
    }

    public void gyroRelativeTurn(int angle, double factor, boolean state, Telemetry t) {

        heading = mrGyro.getHeading();
        error = ((angle + heading) - angle);



        if(error > 0){
            CCW = true;
        }
        else{
            CCW = false;
        }

        while(error != 0 && state) {

            returnGeneralTelemetry(t);

            error = Math.abs((angle + mrGyro.getHeading()) - mrGyro.getHeading());
            pwfactor = (factor * (error/90)+.25);

            if(!CCW){
                left(pwfactor);
            }
            else{
                right(pwfactor);
            }

        }
    }

    public void gyroRelativeTurn(int angle, double factor, boolean ccw, boolean state, Telemetry t) {

        heading = mrGyro.getHeading();
        error = ((angle + heading) - heading);

        CCW = ccw;

        while(error != 0 && state) {

            returnGeneralTelemetry(t);

            error = Math.abs((angle + mrGyro.getHeading()) - mrGyro.getHeading());
            pwfactor = (factor * (error/90)+.25);

            if(!CCW){
                left(pwfactor);
            }
            else{
                right(pwfactor);
            }

        }
    }

}
