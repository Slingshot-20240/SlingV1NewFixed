package org.firstinspires.ftc.teamcode.mechanisms.template;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicSlides {
    private PIDController controller;
    private DcMotorEx slide;
    private static double p, i, d; //has to be tuned
    private static double f; //usually mass moved * constant G
    private int targetPos = 0;

    public BasicSlides(HardwareMap hardwareMap, String configName, int direction, double inP, double inI, double inD, double inF){
        slide = hardwareMap.get(DcMotorEx.class, configName);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if(direction == 0){
            slide.setDirection(DcMotorEx.Direction.FORWARD);
        }else{
            slide.setDirection(DcMotorEx.Direction.REVERSE);
        }

        controller = new PIDController(p,i,d);

        p = inP; i = inI; d = inD; f = inF;
    }

    public void moveTicks(int target){
        targetPos = target;
    }

    public void update(){
        controller.setPID(p,i,d);
        int pos = slide.getCurrentPosition();
        double pid = controller.calculate(pos, targetPos);
        double power = pid + f;
        slide.setPower(power);
    }

    public void changePIDF(double inP, double inI, double inD, double inF){
        p = inP; i = inI; d = inD; f = inF;
    }
    public int getPos(){
        return slide.getCurrentPosition();
    }
}