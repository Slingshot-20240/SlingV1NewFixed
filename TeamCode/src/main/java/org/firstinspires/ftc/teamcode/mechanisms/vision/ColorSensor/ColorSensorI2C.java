package org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorI2C {
    Telemetry telemetry;
    RevColorSensorV3 sensor;
    double blockDistance;

    public ColorSensorI2C(Telemetry t, HardwareMap hm){
        this.telemetry = t;
        this.sensor = hm.get(RevColorSensorV3.class, "colorSensor");
    }
    public ColorSensorI2C(Telemetry t, HardwareMap hm, boolean isBlue){
        this.telemetry = t;
        this.sensor = hm.get(RevColorSensorV3.class, "colorSensor");
    }
    public SampleTypes checkSample(){
        double[] sensorVals = new double[3];
        sensorVals[0] = sensor.red();
        sensorVals[1] = sensor.green();
        sensorVals[2] = sensor.blue();
        SampleTypes best = null;
        double least = Double.MAX_VALUE;
        for(SampleTypes s : SampleTypes.values()){
            if(Math.abs(sensorVals[0]-s.color[0])+Math.abs(sensorVals[1]-s.color[1])+Math.abs(sensorVals[2]-s.color[2]) < least){
                least = Math.abs(sensorVals[0]-s.color[0])+Math.abs(sensorVals[1]-s.color[1])+Math.abs(sensorVals[2]-s.color[2]);
                best = s;
            };
        }
        return best;
    }
    public double[] sensorVals(){
        return new double[]{sensor.red(), sensor.green(), sensor.blue(), sensor.getDistance(DistanceUnit.CM)};
    }
    //TODO: implement HSV based if this isn't consistent
    public boolean hasPixel(){
        if(blockDistance*0.95 > sensor.getDistance(DistanceUnit.CM)) {
            //this code runs if the block is closer
            return true;
        } else if (blockDistance*1.05 > sensor.getDistance(DistanceUnit.CM)) {
            //this code runs if the closest thing is further than the percieved block.
            telemetry.addData("sensor greater diff", sensor.getDistance(DistanceUnit.CM)-blockDistance);
        }
        return false;
    }

    public enum SampleTypes{
        NONE(new double[]{57,95,114}, "NONE"),
        YELLOW(new double[]{215,287,110}, "YELLOW"),
        BLUE(new double[]{55,99,156}, "BLUE"),
        RED(new double[]{173,126,85}, "RED");
        public final double[] color;
        public final String name;
        SampleTypes(double[] color, String name){
            this.color = color;
            this.name = name;
        }
        @NonNull
        public String toString(){
            return name;
        }
    }
}
