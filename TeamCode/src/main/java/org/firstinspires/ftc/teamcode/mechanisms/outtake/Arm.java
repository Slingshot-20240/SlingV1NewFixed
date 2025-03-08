package org.firstinspires.ftc.teamcode.mechanisms.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public Servo armPivot;
    public Servo wrist;
    public Servo claw;

    public Arm(HardwareMap hardwareMap) {
        armPivot = hardwareMap.get(Servo.class, "armPivot");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        claw = hardwareMap.get(Servo.class, "claw");
    }

    // this is for J-Unit testing only
    public Arm(Servo armPivot, Servo wrist, Servo claw) {
        this.armPivot = armPivot;
        this.wrist = wrist;
        this.claw = claw;
    }


    public void moveArm(double armPos, double wristPos) {
        armPivot.setPosition(armPos);
        wrist.setPosition(wristPos);
    }

    public void toScoreSample() {
        moveArm(OuttakeConstants.ArmPositions.BASKET.getArmPos(), OuttakeConstants.ArmPositions.BASKET.getWristPos());
    }

    public void toScoreSpecimen() {
        moveArm(OuttakeConstants.ArmPositions.SPECIMEN_HIGH_RACK.getArmPos(), OuttakeConstants.ArmPositions.SPECIMEN_HIGH_RACK.getWristPos());
    }

    public void readyForTransfer() {
        moveArm(OuttakeConstants.ArmPositions.RETRACTED.getArmPos(), OuttakeConstants.ArmPositions.RETRACTED.getWristPos());
    }

    public void pickSpec() {
        moveArm(OuttakeConstants.ArmPositions.GRABBING_SPEC.getArmPos(), OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
    }

    public void openClaw() {
        claw.setPosition(OuttakeConstants.ArmPositions.RETRACTED.getClawPos());
    }

    public void pullBackToGoUp() {
        moveArm(OuttakeConstants.ArmPositions.TRANSFERING.getArmPos(), OuttakeConstants.ArmPositions.TRANSFERING.getWristPos());
    }

    public void closeClaw() {
        claw.setPosition(OuttakeConstants.ArmPositions.BASKET.getClawPos());
    }

    public void resetHardware() {
        moveArm(OuttakeConstants.ArmPositions.TRANSFERING.getArmPos(), OuttakeConstants.ArmPositions.RETRACTED.getWristPos());
    }

    public void toSafeSampleScore() {
        moveArm(OuttakeConstants.ArmPositions.SAFE_SAMP.getArmPos(), OuttakeConstants.ArmPositions.SAFE_SAMP.getWristPos());
    }
}
