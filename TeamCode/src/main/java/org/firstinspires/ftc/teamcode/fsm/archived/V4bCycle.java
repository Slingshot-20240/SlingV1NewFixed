package org.firstinspires.ftc.teamcode.fsm.archived;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class V4bCycle {
    private Intake intake;
    private Outtake outtake;
    private GamepadMapping controls;
    private Robot robot;

    private V4bCycle.TransferState transferState;
    private Telemetry telemetry;
    private ElapsedTime loopTime;
    private double startTime;

    public V4bCycle(Telemetry telemetry, GamepadMapping controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.outtake = robot.outtake;
        this.controls = controls;

        this.telemetry = telemetry;

        transferState = V4bCycle.TransferState.BASE_STATE;

        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        startTime = loopTime.milliseconds();
    }

    public void v4bActiveUpdate() {
        controls.v4bActiveUpdate();
        robot.drivetrain.update();

        switch (transferState) {
            case BASE_STATE:
                robot.hardwareSoftReset();
                transferState = V4bCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                break;
            case EXTENDO_FULLY_RETRACTED:
                // have to constantly set power of slide motors back
                outtake.returnToRetracted();
                if (controls.extend.value()) {
                    transferState = V4bCycle.TransferState.EXTENDO_FULLY_EXTENDED;
                    intake.extendoFullExtend();
                }
                // TODO: transfer needs to be automatic
//                else if (controls.transfer.value()) {
//                    transferState = TransferState.TRANSFERING;
//                }
                else if (controls.highBasket.value()) {
                    transferState = V4bCycle.TransferState.HIGH_BASKET;
                } else if (controls.lowBasket.value()) {
                    transferState = V4bCycle.TransferState.LOW_BASKET;
                } else if (controls.L1hang.value()) {
                    transferState = V4bCycle.TransferState.HANGING;
                }
                if (controls.transferHover.value()) {
                    intake.v4bActiveIntake.toHovering();
                }
                if (controls.pivot.locked()) {
                    transferState = V4bCycle.TransferState.INTAKING;
                }
                break;
            case EXTENDO_FULLY_EXTENDED:
                outtake.returnToRetracted();
                if (!controls.extend.value()) {
                    intake.v4bActiveIntake.toTransfer();
                    // intake.v4bActiveIntake.transferOff();
                    intake.extendoFullRetract();
                    transferState = V4bCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                }
                if (controls.intakeOnToIntake.locked() || controls.toClear.locked()) {
                    transferState = V4bCycle.TransferState.INTAKING;
                }
                if (controls.botToBaseState.value()) {
                    transferState = V4bCycle.TransferState.BASE_STATE;
                }
                if (controls.pivot.locked()) {
                    intake.v4bActiveIntake.toIntaking();
                    transferState = V4bCycle.TransferState.INTAKING;
                }
                break;
            case INTAKING:
                outtake.returnToRetracted();
                if (!controls.extend.value()) {
                    transferState = V4bCycle.TransferState.EXTENDO_FULLY_EXTENDED;
                } else if (controls.intakeOnToIntake.locked()) {
                    intake.v4bActiveIntake.motorRollerOnToIntake();
                    // intake.activeIntake.backRollerIdle();
                } else if (controls.toClear.locked()) {
                    intake.v4bActiveIntake.motorRollerOnToClear();
                } else if (!controls.intakeOnToIntake.locked() || !controls.toClear.locked()) {
                    intake.v4bActiveIntake.motorRollerOff();
                    // intake.v4bActiveIntake.transferOff();
                }
                if (!controls.pivot.value()) {
                    intake.v4bActiveIntake.toHovering();
                }
                break;
            case TRANSFERING:
                // this is automatic
                outtake.returnToRetracted();
                intake.v4bActiveIntake.toTransfer();
                intake.extendoFullRetract();
                // intake.v4bActiveIntake.transferSample();
                break;
            case HIGH_BASKET:
                // TODO with new outtake
                intake.v4bActiveIntake.toTransfer();
                outtake.extendToHighBasket();
                if (controls.flipBucket.value()) {
                    outtake.bucketDeposit();
                }
                if (!controls.highBasket.value()) {
                    transferState = V4bCycle.TransferState.SLIDES_RETRACTED;
                    break;
                }
                break;
            case LOW_BASKET:
                // TODO with new outtake
                intake.v4bActiveIntake.toTransfer();
                outtake.extendToLowBasket();
                if (controls.flipBucket.value()) {
                    outtake.bucketDeposit();
                }
                if (!controls.lowBasket.value()) {
                    transferState = V4bCycle.TransferState.SLIDES_RETRACTED;
                }
                break;
            case SLIDES_RETRACTED:
                controls.flipBucket.set(false);

                outtake.bucketToReadyForTransfer();
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                transferState = V4bCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                break;
            case HANGING:
                outtake.hang();
                if (!controls.L1hang.value()) {
                    transferState = V4bCycle.TransferState.SLIDES_RETRACTED;
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
        SLIDES_RETRACTED("SLIDES_RETRACTED"),
        HIGH_BASKET("HIGH_BASKET"),
        LOW_BASKET("LOW_BASKET"),
        PUSH_OUT_BAD_COLOR("PUSH_OUT_BAD_COLOR"),
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

