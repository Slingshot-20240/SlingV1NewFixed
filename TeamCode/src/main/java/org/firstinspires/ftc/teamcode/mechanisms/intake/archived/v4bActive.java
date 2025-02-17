package org.firstinspires.ftc.teamcode.mechanisms.intake.archived;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class v4bActive {
    // HARDWARE
    // -----------
    public DcMotorEx rollerMotor;
    public Servo v4b;

    // OTHER
    // ----------
    private GamepadMapping controls;
    private Telemetry telemetry;

    public v4bActive(HardwareMap hwMap, Telemetry telemetry, GamepadMapping controls) {
        rollerMotor = hwMap.get(DcMotorEx.class, "rollerMotor");
        v4b = hwMap.get(Servo.class, "v4b");

        this.controls = controls;
        this.telemetry = telemetry;
    }

    public void toTransfer() {
        v4b.setPosition(IntakeConstants.v4bActiveStates.TRANSFER.v4bPos());
    }

    public void toHovering() {
        v4b.setPosition(IntakeConstants.v4bActiveStates.HOVERING.v4bPos());
    }

    public void toIntaking() {
        v4b.setPosition(IntakeConstants.v4bActiveStates.FULLY_EXTENDED.v4bPos());
    }

    public void motorRollerOnToIntake() {
        rollerMotor.setPower(-1);
    }
    public void motorRollerOff() {
        rollerMotor.setPower(0);
    }
    public void motorRollerOnToClear() { rollerMotor.setPower(0.5); }


}
