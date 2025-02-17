package org.firstinspires.ftc.teamcode.fsm.archived;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.archived.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class ClawCycle {
    private Robot robot;
    private Intake intake;
    private Outtake outtake;
    private GamepadMapping controls;
    private Claw claw;
    public ClawCycle.TransferState transferState;
    private Telemetry telemetry;
    private ElapsedTime loopTime;
    private double startTime;

    // TODO edit wrist movement

    public ClawCycle(Telemetry telemetry, GamepadMapping controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.outtake = robot.outtake;
        this.claw = intake.claw;
        this.controls = controls;

        this.telemetry = telemetry;

        transferState = ClawCycle.TransferState.BASE_STATE;

        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        startTime = loopTime.milliseconds();
    }
    public void clawIntakeUpdate() {
        controls.update();
        robot.drivetrain.update();

        switch (transferState) {
            case BASE_STATE:
                robot.hardwareSoftReset();
                transferState = ClawCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                break;
            case EXTENDO_FULLY_RETRACTED:
                // have to constantly set power of slide motors back
                outtake.returnToRetracted();

                if (controls.extend.value()) {
                    transferState = ClawCycle.TransferState.HOVERING;
                    intake.extendoFullExtend();
                }
                else if (controls.highBasket.value()) {
                    transferState = ClawCycle.TransferState.HIGH_BASKET;
                } else if (controls.lowBasket.value()) {
                    transferState = ClawCycle.TransferState.LOW_BASKET;
                } else if (controls.L1hang.value()) {
                    transferState = ClawCycle.TransferState.HANGING;
                }
                if (controls.transferHover.value()) {
                    transferState = TransferState.HOVERING;
                }
                break;
            case INTAKING:
                outtake.returnToRetracted();
                claw.moveToPickingSample();
                if (!controls.extend.value()) {
                    transferState = TransferState.TRANSFERING;
                    startTime = loopTime.milliseconds();
                }
                if (!controls.pivot.locked()) {
                    transferState = TransferState.HOVERING;
                } else if (controls.pivot.locked()) {
                    claw.moveToPickingSample();
                    if (controls.openClaw.value()) {
                        claw.openClaw();
                    } else {
                        claw.closeClaw();
                    }
                }
                break;
            case TRANSFERING:
                outtake.returnToRetracted();
                robot.intake.extendoFullRetract();
                claw.moveToTransfer();
                claw.turnWristToTransfer();
                if (loopTime.milliseconds() - startTime >= 1000){
                    claw.openClaw();
                    controls.extend.set(false);
                    controls.transferHover.set(false);
                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                }
                break;
            case HOVERING:
                controls.transferHover.set(true);
                outtake.returnToRetracted();
                claw.moveToHovering();
                claw.controlWristPos();
                if (controls.pivot.locked()) {
                    transferState = ClawCycle.TransferState.INTAKING;
                }
                if (!controls.transferHover.value()) {
                    transferState = TransferState.TRANSFERING;
                    startTime = loopTime.milliseconds();
                }
                if (!controls.extend.value()) {
                    claw.moveToTransfer();
                    intake.extendoFullRetract();
                    transferState = TransferState.TRANSFERING;
                    startTime = loopTime.milliseconds();
                }
                if (controls.openClaw.value()) {
                    claw.openClaw();
                } else {
                    claw.closeClaw();
                }
                break;
            case HIGH_BASKET:
                outtake.extendToHighBasket();
                claw.closeClaw();
                claw.moveToOuttaking();
                if (controls.flipBucket.value()) {
                    outtake.bucketDeposit();
                }
                if (!controls.highBasket.value()) {
                    transferState = ClawCycle.TransferState.SLIDES_RETRACTED;
                    break;
                }
                break;
            case LOW_BASKET:
                outtake.extendToLowBasket();
                if (controls.flipBucket.value()) {
                    outtake.bucketDeposit();
                }
                if (!controls.lowBasket.value()) {
                    transferState = ClawCycle.TransferState.SLIDES_RETRACTED;
                }
                break;
            case SLIDES_RETRACTED:
                controls.flipBucket.set(false);
                outtake.bucketToReadyForTransfer();
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                transferState = ClawCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                break;
            case HANGING:
                outtake.hang();
                if (!controls.L1hang.value()) {
                    transferState = ClawCycle.TransferState.SLIDES_RETRACTED;
                }
                break;
        }
    }

    public enum TransferState {
        BASE_STATE("BASE_STATE"),
        EXTENDO_FULLY_RETRACTED("EXTENDO_FULLY_RETRACTED"),
        EXTENDO_FULLY_EXTENDED("EXTENDO_FULLY_EXTENDED"),
        INTAKING("INTAKING"),
        TRANSFERING("TRANSFERING"),
        HOVERING("HOVERING"),
        SLIDES_RETRACTED("SLIDES_RETRACTED"),
        HIGH_BASKET("HIGH_BASKET"),
        LOW_BASKET("LOW_BASKET"),
        HANGING("HANGING");

        private String state;

        TransferState(String stateName) {
            state = stateName;
        }

        public String stateName() {
            return state;
        }
    }
}
