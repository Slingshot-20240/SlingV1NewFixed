package org.firstinspires.ftc.teamcode.mechanisms.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeachArm {

    DcMotor armMotor;
    Servo claw;

    public TeachArm(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        this.claw = hardwareMap.get(Servo.class, "claw");
    }

    public void openClaw() {
        claw.setPosition(1);
    }

}
