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
    ColorRangeSensor sensor;
    double blockDistance = 3.81;
    public boolean isBlue;
    public ColorSensorI2C(HardwareMap hm){
        this.sensor = hm.get(RevColorSensorV3.class, "colorSensor");
    }
    public ColorSensorI2C( HardwareMap hm, boolean isBlue){
        this.sensor = hm.get(RevColorSensorV3.class, "colorSensor");
        this.isBlue = isBlue;
    }
    // this is for testing only
    public ColorSensorI2C (ColorRangeSensor colorSensor, boolean isBlue) {
        this.sensor = colorSensor;
        this.isBlue = isBlue;
    }
    public void setIsBlue(boolean b){
        this.isBlue = b;
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
    public boolean hasSample(){
        //this code runs if the block is closer
        return blockDistance * 0.95 > sensor.getDistance(DistanceUnit.CM);
    }

    public enum SampleTypes{
        YELLOW(new double[]{306,409,113}, "YELLOW"),
        BLUE(new double[]{55,110,274}, "BLUE"),
        RED(new double[]{187,115,77}, "RED"),
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
