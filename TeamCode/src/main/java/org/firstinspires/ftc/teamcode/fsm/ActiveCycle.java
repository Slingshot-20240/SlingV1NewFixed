package org.firstinspires.ftc.teamcode.fsm;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class ActiveCycle {
    // mechanisms
    private final Intake intake;
    private final Outtake outtake;
    private final GamepadMapping controls;
    private final Robot robot;
    private ActiveCycle.TransferState transferState;
    private final Telemetry telemetry;
    private final ElapsedTime loopTime;
    private double startTime;
    private final Arm arm;
    public ActiveCycle(Telemetry telemetry, GamepadMapping controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.outtake = robot.outtake;
        this.controls = controls;
        this.arm = robot.arm;

        this.telemetry = telemetry;

        transferState = TransferState.BASE_STATE;

        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        startTime = loopTime.milliseconds();
    }
    public void activeIntakeUpdate() {
        controls.update();
        //robot.drivetrain.update();

        if(outtake.touchSensor.isPressed()) {
            outtake.resetEncoders();
        }

        switch (transferState) {
            case BASE_STATE:
                robot.hardwareSoftReset();
                transferState = ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                break;
            case EXTENDO_FULLY_RETRACTED:
                // have to constantly set power of slide motors back
                outtake.returnToRetracted();
                controls.openClaw.set(false);
                if (controls.extend.value()) {
                    transferState = ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                } else if (controls.transfer.locked()) {
                    transferState = TransferState.TRANSFERING;
                } else if (controls.highBasket.value()) {
                    transferState = ActiveCycle.TransferState.HIGH_BASKET;
                    startTime = loopTime.milliseconds();
                } else if (controls.lowBasket.value()) {
                    transferState = ActiveCycle.TransferState.LOW_BASKET;
                    startTime = loopTime.milliseconds();
                }
                if (controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.motorRollerOnToIntake();
                } else {
                    intake.activeIntake.motorRollerOff();
                }
                if (controls.specMode.value()) {
                    transferState = TransferState.SPEC_MODE;
                }
                break;
            case EXTENDO_FULLY_EXTENDED:
                controls.resetSpecControls();
                outtake.returnToRetracted();
                intake.extendoFullExtend();
                if (!controls.extend.value()) {
                    intake.extendForOuttake();
                    intake.activeIntake.flipToTransfer();
                    controls.transfer.set(false);
                    controls.resetOuttakeControls();
                    arm.retract();
                    arm.openClaw();
                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                }
                if (controls.intakeOnToIntake.locked() || controls.toClear.locked() || controls.clearSpec.locked()) {
                    transferState = TransferState.INTAKING;
                }
                break;
            case INTAKING:
                outtake.returnToRetracted();
                if (!controls.extend.value()) {
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.clear.set(false);
                } else if (controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                } else if (controls.toClear.locked()) {
                    intake.activeIntake.flipDownToClear();
                    if (controls.clear.value()) {
                        intake.activeIntake.motorRollerOnToClear();
                    } else {
                        intake.activeIntake.motorRollerOff();
                    }
                } else if (controls.clearSpec.locked()) {
                    intake.activeIntake.flipDownFull();
                    if (controls.clear.value()) {
                        intake.activeIntake.motorRollerOnToClear();
                    } else {
                        intake.activeIntake.motorRollerOff();
                    }
                } else if (!controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.flipUp();
                    intake.activeIntake.transferOff();
                } else if (!controls.toClear.locked()) {
                    intake.activeIntake.flipUp();
                    intake.activeIntake.transferOff();
                } else if (!controls.clearSpec.locked()) {
                    intake.activeIntake.flipUp();
                    intake.activeIntake.transferOff();
                }
                break;
            case TRANSFERING:
                outtake.returnToRetracted();
//                if (loopTime.milliseconds() - startTime <= 700){
//                    intake.activeIntake.transferSample();
//
//                    if (controls.extend.value()) {
//                        transferState = ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED;
//                        intake.extendoFullExtend();
//                        controls.transfer.set(false);
//                        break;
//                    }
//                } else if (loopTime.milliseconds() - startTime > 700) {
//                    transferState = ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED;
//                    controls.transfer.set(false);
//                }
                intake.activeIntake.transferSample();
                if (!controls.transfer.locked()) {
                    intake.activeIntake.transferOff();
                    transferState = TransferState.TRANSFER_CLOSE;
                    startTime = loopTime.milliseconds();
                }
                break;
            case TRANSFER_CLOSE:
                if (loopTime.milliseconds() - startTime >= 300) {
                    arm.closeClaw();
                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                    break;
                }
                break;
            case HIGH_BASKET:
                intake.extendForOuttake();

                outtake.extendToHighBasket();
                robot.arm.toScoreSpecimen();


                if (controls.openClaw.value()) {
                    arm.openClaw();
                }
                if (!controls.highBasket.value()) {
                    transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
                }
                if (controls.extend.value()) {
                    intake.extendoFullExtend();
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                }
                break;
            case LOW_BASKET:
                intake.extendForOuttake();
                outtake.extendToHighBasket();
                robot.arm.toScoreSpecimen();

                if (controls.openClaw.value()) {
                    arm.openClaw();
                }
                if (!controls.lowBasket.value()) {
                    transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
                }
                if (controls.extend.value()) {
                    intake.extendoFullExtend();
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                }
                break;
            case SLIDES_RETRACTED:
                controls.resetOuttakeControls();
                controls.resetMultipleControls(controls.transfer, controls.extend, controls.scoreSpec);
                // could also do to base state
                robot.arm.retract();
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                transferState = ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                break;
//            case HANGING:
//                outtake.hang();
//                if (!controls.L1hang.value()) {
//                    outtake.bucketToReadyForTransfer();
//                    transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
//                }
//                break;
//            case PUSH_OUT_BAD_COLOR:
//                if (loopTime.milliseconds() - startTime <= 1000 && loopTime.milliseconds() - startTime >= 0) {
//                    intake.activeIntake.pushOutSample();
//                } else {
//                    intake.activeIntake.transferOff();
//                    transferState = ActiveCycle.TransferState.INTAKING;
//                }
//                break;
            case SPEC_MODE:
                outtake.returnToRetracted();
                intake.extendForOuttake();
                if (loopTime.milliseconds() - startTime >= 300) {
                    arm.pickSpec();
                    transferState = TransferState.SPEC_IDLE;
                    break;
                }
                break;
            case SPEC_IDLE:
                intake.extendoFullRetract();
                if (controls.openClaw.value()) {
                    arm.closeClaw();
                } else if (!controls.openClaw.value()) {
                    arm.openClaw();
                }
                if (controls.scoreSpec.value()) {
                    transferState = TransferState.SPEC_SCORING;
                }
                if (!controls.specMode.value()) {
                    transferState = TransferState.RETURN_TO_SAMPLE_MODE;
                    startTime = loopTime.milliseconds();
                }
                break;
            case RETURN_TO_SAMPLE_MODE:
                outtake.returnToRetracted();
                intake.extendForOuttake();
                if (loopTime.milliseconds() - startTime >= 300) {
                    arm.retract();
                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                    break;
                }

            case SPEC_SCORING:
                outtake.extendToSpecimenHighRack();
                intake.extendForOuttake();
                // if 100ms have passed since telling claw to close, outtake to lowBasket
                if (loopTime.milliseconds() - startTime >= 300) {
                    outtake.extendToSpecimenHighRack();
                    robot.arm.toScoreSpecimen();
                } else if (!controls.scoreSpec.value()) {
                    transferState = TransferState.SPEC_RETRACTING;
                    startTime = loopTime.milliseconds();
                }

                if (controls.highBasket.value()) {
                    robot.arm.toScoreSample();
                    transferState = ActiveCycle.TransferState.HIGH_BASKET;
                    controls.resetMultipleControls(controls.flipBucket, controls.scoreSpec);
                }
                // TODO: add later
//                if (controls.lowBasket.value()) {
//                    outtake.bucketTilt();
//                    transferState = ActiveCycle.TransferState.LOW_BASKET;
//                    controls.resetMultipleControls(controls.flipBucket, controls.scoreSpec);
//                }
                break;
            case SPEC_RETRACTING:
                outtake.returnToRetracted();

                if (loopTime.milliseconds() - startTime <= 400 && loopTime.milliseconds() - startTime >= 200) {
                    arm.openClaw();
                } else if (loopTime.milliseconds() - startTime > 400) {
                    transferState = TransferState.SPEC_IDLE;
                    controls.openClaw.set(false);
                    controls.resetOuttakeControls();
                }
                break;

        }
    }

    public TransferState getState() {
        return transferState;
    }
    public void setState(TransferState newState) {
        transferState = newState;
    }

    public enum TransferState {
        BASE_STATE("BASE_STATE"),
        EXTENDO_FULLY_RETRACTED("EXTENDO_FULLY_RETRACTED"),
        EXTENDO_FULLY_EXTENDED("EXTENDO_FULLY_EXTENDED"),
        INTAKING("INTAKING"),
        TRANSFERING("TRANSFERING"),
        TRANSFER_CLOSE("TRANSFER_CLOSE"),
        SLIDES_RETRACTED("SLIDES_RETRACTED"),
        HIGH_BASKET("HIGH_BASKET"),
        LOW_BASKET("LOW_BASKET"),
        SPEC_SCORING("SPEC_SCORING"),
        SPEC_RETRACTING("SPEC_RETRACTING"),
        SPEC_MODE("SPEC_MODE"),
        RETURN_TO_SAMPLE_MODE("RETURN_TO_SAMPLE_MODE"),
        SPEC_IDLE("SPEC_IDLE"),
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