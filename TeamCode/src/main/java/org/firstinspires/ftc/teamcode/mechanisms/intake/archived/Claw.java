package org.firstinspires.ftc.teamcode.mechanisms.intake.archived;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class Claw {
    // HARDWARE
    public Servo clawServo; // mini axon
    public Servo wrist; // gobilda torque
    public Servo v4b; // max axon (virtual fourbar)

    // OTHER
    private Telemetry telemetry;
    private GamepadMapping controls;
    private ClawConstants.ClawStates clawState = ClawConstants.ClawStates.BASE_STATE;
    private double wristYaw;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry, GamepadMapping controls) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        v4b = hardwareMap.get(Servo.class, "v4b");

        this.telemetry = telemetry;
        this.controls = controls;
    }

    public void moveToTransfer() {
        // v4b at an angle, claw parallel (or close ish)
        // this should move claw to transfer pos, then wrist to transfer pos, then open claw, may need elapsed time if too fast
        v4b.setPosition(ClawConstants.ClawStates.TRANSFER.getV4bPos());
    }

    public void moveToHovering() {
        // v4b at an angle, claw also at an angle (90 angle between v4b and claw)
        v4b.setPosition(ClawConstants.ClawStates.HOVER_OVER_SAMPLES.getV4bPos());
    }

    public void moveToPickingSample() {
        v4b.setPosition(ClawConstants.ClawStates.PICKING_UP_SAMPLE.getV4bPos());
    }

    public void moveToOuttaking() {
        v4b.setPosition(ClawConstants.ClawStates.OUTTAKING.getV4bPos());
    }

    public void resetClaw() {
        wrist.setPosition(ClawConstants.ClawStates.TRANSFER.getWristPos());
        v4b.setPosition(ClawConstants.ClawStates.BASE_STATE.getV4bPos());
        clawServo.setPosition(ClawConstants.ClawStates.PICKING_UP_SAMPLE.getClawPos());
    }

    public void controlWristPos() {
        wristYaw += controls.wristYaw * .01; // * wristSpeed;
        //wristYaw = Range.clip(wristYaw, .1, .6);
        wrist.setPosition(wristYaw);
    }

    public void closeClaw() {
        clawServo.setPosition(ClawConstants.ClawStates.PICKING_UP_SAMPLE.getClawPos());
    }

    public void openClaw() {
        clawServo.setPosition(ClawConstants.ClawStates.HOVER_OVER_SAMPLES.getClawPos());
    }

    public void turnWristToTransfer() {
        wrist.setPosition(ClawConstants.ClawStates.TRANSFER.getWristPos());
    }
}
