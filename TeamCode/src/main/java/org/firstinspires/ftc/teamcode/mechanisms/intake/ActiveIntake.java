package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ActiveIntake {
    // HARDWARE
    // -----------
    public DcMotorEx rollerMotor;
    public Servo pivotAxon;

    public ActiveIntake(HardwareMap hwMap) {
        rollerMotor = hwMap.get(DcMotorEx.class, "rollerMotor");
        pivotAxon = hwMap.get(Servo.class, "pivotAxon");

        rollerMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rollerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pivotAxon.setDirection(Servo.Direction.FORWARD);
    }

    // This is for testing only
    public ActiveIntake(DcMotorEx rollerMotor, Servo pivotAxon) {
        this.rollerMotor = rollerMotor;
        this.pivotAxon = pivotAxon;
    }

    public void flipDownFull() {
        pivotAxon.setPosition(IntakeConstants.ActiveIntakeStates.FULLY_EXTENDED.pivotPos());
    }

    public void flipDownToClear() {
        pivotAxon.setPosition(IntakeConstants.ActiveIntakeStates.CLEARING.pivotPos());
    }

    public void flipToTransfer() {
        pivotAxon.setPosition(IntakeConstants.ActiveIntakeStates.TRANSFER.pivotPos());
    }

    public void flipUp() {
        pivotAxon.setPosition(IntakeConstants.ActiveIntakeStates.TRANSFER.pivotPos());
    }

    public void pivotSpecMode() {
        pivotAxon.setPosition(IntakeConstants.ActiveIntakeStates.SPEC_MODE.pivotPos());
    }

    public void motorRollerOnToIntake() {
        rollerMotor.setPower(-1);
    }
    public void motorRollerOff() {
        rollerMotor.setPower(0);
    }
    public void motorRollerOnToClear() { rollerMotor.setPower(0.55); }

    public void transferSample() {
        // pivotAnalog.runToPos(IntakeConstants.IntakeState.TRANSFER.pivotPos());
        rollerMotor.setPower(1);
    }

    public void clearIntake() {
        motorRollerOnToClear();
    }
}

