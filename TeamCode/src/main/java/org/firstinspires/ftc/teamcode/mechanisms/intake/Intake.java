package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    // HARDWARE
    // -----------
    public ActiveIntake activeIntake;
    public Servo leftExtendo; // axon
    public Servo rightExtendo; // axon

    public Intake(HardwareMap hwMap) {
        activeIntake = new ActiveIntake(hwMap);

        rightExtendo = hwMap.get(Servo.class, "rightLinkage");
        leftExtendo = hwMap.get(Servo.class, "leftLinkage");
    }

    // This is for testing only :)
    public Intake(Servo rightExtendo, Servo leftExtendo, ActiveIntake activeIntake) {
        this.rightExtendo = rightExtendo;
        this.leftExtendo = leftExtendo;
        this.activeIntake = activeIntake;
    }

    public void extendoFullExtend() {
        rightExtendo.setPosition(IntakeConstants.ActiveIntakeStates.FULLY_EXTENDED.rLinkagePos());
        leftExtendo.setPosition(IntakeConstants.ActiveIntakeStates.FULLY_EXTENDED.lLinkagePos());
    }

    public void extendoFullRetract() {
        rightExtendo.setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.rLinkagePos());
        leftExtendo.setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.lLinkagePos());
    }

    public void extendToTransfer() {
        rightExtendo.setPosition(IntakeConstants.ActiveIntakeStates.TRANSFER.rLinkagePos());
        leftExtendo.setPosition(IntakeConstants.ActiveIntakeStates.TRANSFER.lLinkagePos());
    }

    public void extendForSpecMode() {
        rightExtendo.setPosition(IntakeConstants.ActiveIntakeStates.SPEC_MODE.rLinkagePos());
        leftExtendo.setPosition(IntakeConstants.ActiveIntakeStates.SPEC_MODE.lLinkagePos());
    }

    public void resetHardware() {
        activeIntake.motorRollerOff();
        activeIntake.rollerMotor.setDirection(DcMotorEx.Direction.REVERSE);

        activeIntake.flipUp();

        extendoFullRetract();
    }
}
