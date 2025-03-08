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
    public double[] getVals(){
        double[] result = limelight.getLatestResult().getPythonOutput();
        // form: [EXTEND BY, TRANSLATE BY]

        //EXTENDING
        double nY = -(result[0]-240)/240*181.218240428; //get n
        nY = 65-Math.atan(nY/472.089656463);  //get the angle
        nY = 199.51371*Math.tan(nY); //get ll dist

        //TRANLATING
        double nX = -(result[1]-320)/320*220.361624027;

        return new double[]{nY-50.8, 107.95+nX};
    }
    public void setColors(boolean getRed, boolean getBlue, boolean getYellow){
        limelight.updatePythonInputs(new double[]{getRed?1.0:0.0, getBlue?1.0:0.0, getYellow?1.0:0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
    public void setColors(boolean[] colors){
        limelight.updatePythonInputs(new double[]{colors[0]?1.0:0.0, colors[1]?1.0:0.0, colors[2]?1.0:0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
}
