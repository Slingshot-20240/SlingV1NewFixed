package org.firstinspires.ftc.teamcode.mechanisms.misc;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class AnalogServo {
    public AnalogInput analogInput;
    public Servo analogServo;
    private PIDController controller;
    private double p = 0, i = 0, d = 0; //has to be tuned
    private double f = 0;

    public AnalogServo(Servo analogServo, AnalogInput analogInput, double p, double i, double d, double f) {
        this.analogServo = analogServo;
        this.analogInput = analogInput;
        this.controller = new PIDController(p, i, d);
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public double powerToPosition(double power) {
        return power * 2 - 1;
    }

    public void setPower(double power) {
        analogServo.setPosition(powerToPosition(power));
    }

    public void runToPos(double targetAngle) {
        controller.setPID(p, i, d);
        double pos = this.getPosition();
        double pid = controller.calculate(pos, targetAngle);
        double ff = Math.cos(Math.toRadians(targetAngle)) * f;
        double power = pid + ff;

        analogServo.setPosition(powerToPosition(power));
    }

    public double getPosition() {
        return analogInput.getVoltage() / 3.3 * 360;
    }

    public void changePIDF(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }
}