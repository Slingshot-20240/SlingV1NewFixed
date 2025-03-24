package org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorI2C {
    public ColorRangeSensor sensor;
    double blockDistance = 3.81;
    public static boolean isBlue;

    private double rThresh = 0.7;
    private double bThresh = 2.1;
    private double yThresh = 0.9;

    public ColorSensorI2C(HardwareMap hm){
        this.sensor = hm.get(RevColorSensorV3.class, "colorSensor");
    }
    public ColorSensorI2C( HardwareMap hm, boolean isBlueYub){
        this.sensor = hm.get(RevColorSensorV3.class, "colorSensor");
        isBlue = isBlueYub;
    }
    // this is for testing only
    public ColorSensorI2C (ColorRangeSensor colorSensor, boolean isBlue) {
        this.sensor = colorSensor;
        this.isBlue = isBlue;
    }
    public void setIsBlue(boolean b){
        isBlue = b;
    }
    public boolean opposingColor(){
        if (isBlue && checkSample().equals(SampleTypes.RED) && !checkSample().equals(SampleTypes.YELLOW)) {
            return true;
        } else if (!isBlue && checkSample().equals(SampleTypes.BLUE) && !checkSample().equals(SampleTypes.YELLOW)) {
            return true;
        }
        return false;
    }
    public boolean teamColor(){
        return (!isBlue?SampleTypes.RED:SampleTypes.BLUE).equals(checkSample());
    }
    public SampleTypes checkSample(){
//        double[] sensorVals = new double[3];
//        sensorVals[0] = sensor.red();
//        sensorVals[1] = sensor.green();
//        sensorVals[2] = sensor.blue();
//        SampleTypes best = null;
//        double least = Double.MAX_VALUE;
//        for(SampleTypes s : SampleTypes.values()){
//            if(Math.abs(sensorVals[0]-s.color[0])+Math.abs(sensorVals[1]-s.color[1])+Math.abs(sensorVals[2]-s.color[2]) < least){
//                least = Math.abs(sensorVals[0]-s.color[0])+Math.abs(sensorVals[1]-s.color[1])+Math.abs(sensorVals[2]-s.color[2]);
//                best = s;
//            };
//        }
//        return best;

        if (sensor.red()/sensor.blue() > rThresh && sensor.red()/sensor.green() > yThresh) {
            return SampleTypes.RED;
        } else if (sensor.blue()/sensor.red() > bThresh) {
            return SampleTypes.BLUE;
        } else if (sensor.red()/sensor.blue() > rThresh && sensor.red()/sensor.green() < yThresh) {
            return SampleTypes.YELLOW;
        }
        return SampleTypes.NONE;
    }
    public void changeThres(double rThresh, double bThresh, double yThresh) {
        this.rThresh = rThresh;
        this.bThresh = bThresh;
        this.yThresh = yThresh;
    }
    public double[] sensorVals(){
        return new double[]{sensor.red(), sensor.green(), sensor.blue(), sensor.getDistance(DistanceUnit.CM)};
    }
    //TODO: implement HSV based if this isn't consistent
    public boolean hasSample(){
        //this code runs if the block is closer
        return blockDistance * 0.95 > sensor.getDistance(DistanceUnit.CM);
    }

    public enum SampleTypes{
        YELLOW(new double[]{370,424,108}, "YELLOW"),
        BLUE(new double[]{47,96,251}, "BLUE"),
        RED(new double[]{230,109,58}, "RED"),
        NONE(new double[]{26,49,43}, "NONE");
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
