package org.firstinspires.ftc.teamcode.teleop.testers.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

@TeleOp
@Disabled
public class SensorColorCalibration extends OpMode {
    int iterationCount = 0;
    double[] currentMean = new double[3];
    ColorRangeSensor sensor;
    List<Object> test;
    @Override
    public void init() {
        sensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
    }

    @Override
    public void loop() {
        double[] byeLoopTime = new double[3];
        telemetry.clearAll();
        //uh, I feel like i had to add brackets, this is pretty much unnessesary...
        if(sensor != null){

            currentMean[0] = (sensor.red()+currentMean[0]*iterationCount)/(iterationCount+1);
            currentMean[1] = (sensor.green()+currentMean[1]*iterationCount)/(iterationCount+1);
            currentMean[2] = (sensor.blue()+currentMean[2]*iterationCount)/(iterationCount+1);

            byeLoopTime[0] = (sensor.red());
            byeLoopTime[1] = (sensor.green());
            byeLoopTime[2] = (sensor.blue());
            iterationCount++;

        }
        //You can reset the current set using the options key!
        if(gamepad1.options){
            currentMean = new double[3];
            iterationCount = 0;
        }
        telemetry.addData("dist: ", sensor.getDistance(DistanceUnit.CM)); //see if this changes when a pixel is placed in. this is used to verify if there is a pixel or not
        telemetry.addData("mean:", parseDoubleArray(currentMean));
        telemetry.addData("current Val:", parseDoubleArray(byeLoopTime));
    }
    public String parseDoubleArray(double[] array){
        //I chose to do this to sacrifice a little bit of memory for a little bit of speed :)
        int len = array.length;
        String s = "";
        for(int i = 0;i<len;i++){
            s += (int) array[i]+", ";
        }
        return s.substring(0,s.length()-2);
    }
}
