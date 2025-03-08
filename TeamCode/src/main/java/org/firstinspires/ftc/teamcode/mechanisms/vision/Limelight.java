package org.firstinspires.ftc.teamcode.mechanisms.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    Limelight3A limelight;
    public Limelight(HardwareMap hardwareMap, boolean getRed, boolean getBlue, boolean getYellow){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.updatePythonInputs(new double[]{getRed?1.0:0.0, getBlue?1.0:0.0, getYellow?1.0:0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        limelight.start();
    }
    public double[] location(){
        //form: x-coord, y-coord, long edge length, short edge length, angle
        LLResult result = limelight.getLatestResult();
        if(result.getPythonOutput().length<1)
            return null;
        return result.getPythonOutput();

    }
    public void setColors(boolean getRed, boolean getBlue, boolean getYellow){
        limelight.updatePythonInputs(new double[]{getRed?1.0:0.0, getBlue?1.0:0.0, getYellow?1.0:0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
    public void setColors(boolean[] colors){
        limelight.updatePythonInputs(new double[]{colors[0]?1.0:0.0, colors[1]?1.0:0.0, colors[2]?1.0:0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
}
