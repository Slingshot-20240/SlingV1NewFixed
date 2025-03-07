package org.firstinspires.ftc.teamcode.fsm;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorModule;
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
    private final ColorSensorModule colorSensor;

    private boolean safeDeposit = false;

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
        this.colorSensor = robot.colorSensor;
    }
    public void activeIntakeUpdate() {

        telemetry.addData("transfer state", transferState.toString());

        if(outtake.touchSensor.isPressed()) {
            outtake.resetEncoders();
        }

        if(controls.botToBaseState.value()) {
            controls.botToBaseState.set(false);
            transferState = TransferState.BASE_STATE;
            controls.resetAllControls();
        }

        if (controls.safeDeposit.value()) {
            safeDeposit = true;
        }

        switch (transferState) {
            case BASE_STATE:
                robot.hardwareSoftReset();
                transferState = ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                break;
            case EXTENDO_FULLY_RETRACTED:
                // have to constantly set power of slide motors back
                outtake.returnToRetracted();
                //intake.extendoFullRetract();
                controls.openClaw.set(false);
                if (controls.extend.value()) {
                    transferState = ActiveCycle.TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                } else if (controls.transfer.value()) {
                    transferState = TransferState.TRANSFERING;
                } else if (controls.highBasket.value()) {
                    transferState = ActiveCycle.TransferState.HIGH_BASKET;
                    robot.arm.pullBackToGoUp();
                    startTime = loopTime.milliseconds();
                } else if (controls.lowBasket.value()) {
                    transferState = ActiveCycle.TransferState.LOW_BASKET;
                    robot.arm.pullBackToGoUp();
                    startTime = loopTime.milliseconds();
                }
                if (controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.motorRollerOnToIntake();
                } else {
                    intake.activeIntake.motorRollerOff();
                }
                if (controls.specMode.value()) {
                    transferState = TransferState.SPEC_MODE;
                    startTime = loopTime.milliseconds();
                }
                if (controls.hang.value()) {
                    transferState = TransferState.HANGING;
                    outtake.hang();
                }
                break;
            case EXTENDO_FULLY_EXTENDED:
                controls.resetSpecControls();
                outtake.returnToRetracted();
                intake.extendoFullExtend();
                if (!controls.extend.value() && !controls.specMode.value()) {
                    intake.extendToTransfer();
                    intake.activeIntake.flipToTransfer();
                    controls.transfer.set(false);
                    controls.resetOuttakeControls();
                    arm.readyForTransfer();
                    arm.openClaw();
                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                    //intake.extendoFullRetract();
                } else if (!controls.extend.value() && controls.specMode.value()) {

                    transferState = TransferState.SPEC_IDLE;
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
                    controls.intakeOnToIntake.set(false);
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

                if (colorSensor.opposingColor()) {
                    intake.activeIntake.clearIntake();
                    transferState = TransferState.WRONG_COLOR;
                    //startTime = loopTime.milliseconds();
                }
                break;
            case WRONG_COLOR:
                if (!colorSensor.opposingColor()) {
                    //if (startTime - loopTime.milliseconds() >= 200) {
                        intake.activeIntake.motorRollerOff();
                        transferState = TransferState.INTAKING;
                        //break;
                    //}
                }
                break;
            case TRANSFERING:
                outtake.returnToRetracted();
                if(controls.transfer.value()) {
                    intake.activeIntake.transferSample();
                } else {
                    intake.activeIntake.transferOff();
                }
                if (controls.openClaw.value()) {
                    arm.closeClaw();
                } else {
                    arm.openClaw();
                }
                if (controls.highBasket.value()) {
                    arm.closeClaw();
                    controls.openClaw.set(false);
                    controls.transfer.set(false);
                    transferState = ActiveCycle.TransferState.HIGH_BASKET;
                    robot.arm.pullBackToGoUp();
                    startTime = loopTime.milliseconds();
                }
                if (controls.lowBasket.value()) {
                    arm.closeClaw();
                    controls.openClaw.set(false);
                    controls.transfer.set(false);
                    transferState = ActiveCycle.TransferState.LOW_BASKET;
                    robot.arm.pullBackToGoUp();
                    startTime = loopTime.milliseconds();
                }
                if (controls.extend.value()) {
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetMultipleControls(controls.openClaw, controls.lowBasket, controls.highBasket, controls.transfer);
                }
                if (controls.intakeOnToIntake.locked() && !controls.transfer.locked()) {
                    intake.activeIntake.motorRollerOnToIntake();
                } else if (controls.intakeOnToIntake.locked() && !controls.transfer.locked()){
                    intake.activeIntake.motorRollerOff();
                }

                break;
//            case TRANSFER_CLOSE:
//                if (loopTime.milliseconds() - startTime >= 200) {
//                    arm.closeClaw();
//                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
////                    intake.extendoFullRetract();
//                    break;
//                }
//                break;
            case HIGH_BASKET:
                intake.extendToTransfer();
                outtake.extendToHighBasket();
                intake.activeIntake.transferOff();
                if (loopTime.milliseconds() - startTime >= 300) {
                    if (safeDeposit) {
                        robot.arm.toSafeSampleScore();
                    } else {
                        robot.arm.toScoreSample();
                    }
                }

                if (controls.openClaw.value()) {
                    arm.openClaw();
                }
                if (!controls.highBasket.value()) {
                    transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
                }
                if (controls.extend.value()) {
                    intake.extendoFullExtend();
                    arm.pullBackToGoUp();
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                }
                break;
            case LOW_BASKET:
                intake.extendToTransfer();
                outtake.extendToLowBasket();
                intake.activeIntake.transferOff();
                if (loopTime.milliseconds() - startTime >= 300) {
                    if (safeDeposit) {
                        robot.arm.toSafeSampleScore();
                    } else {
                        robot.arm.toScoreSample();
                    }
                }

                if (controls.openClaw.value()) {
                    arm.openClaw();
                }
                if (!controls.lowBasket.value()) {
                    transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
                }
                if (controls.extend.value()) {
                    intake.extendoFullExtend();
                    arm.pullBackToGoUp();
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                }
                break;
            case SLIDES_RETRACTED:
                controls.resetOuttakeControls();
                controls.resetMultipleControls(controls.transfer, controls.extend, controls.scoreSpec, controls.openClaw, controls.intakeOnToIntake);
                // could also do to base state
                robot.arm.pullBackToGoUp();
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                transferState = ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                intake.activeIntake.transferOff();
                break;
            case HANGING:
                // high pos
                if (!controls.hang.value()) {
                    outtake.moveTicks(OuttakeConstants.SlidePositions.HANGING_LOW.getSlidePos());
                    //transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
                }
                break;
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
                intake.extendForSpecMode();
                if (loopTime.milliseconds() - startTime >= 500 && loopTime.milliseconds() - startTime <= 1000 ) {
                    arm.pickSpec();
                } else if (loopTime.milliseconds() - startTime > 1000) {
                    transferState = TransferState.SPEC_IDLE;
                    break;
                }
                break;
            case SPEC_IDLE:
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                intake.activeIntake.transferOff();
                intake.activeIntake.pivotAxon.setPosition(IntakeConstants.ActiveIntakeStates.FAILSAFE_CLEARING.pivotPos());
                arm.pickSpec();

                if (controls.openClaw.value()) {
                    arm.closeClaw();
                } else if (!controls.openClaw.value()) {
                    arm.openClaw();
                }
                if (controls.scoreSpec.value()) {
                    outtake.extendToSpecimenHighRackLow();
                    transferState = TransferState.SPEC_SCORING;
                    startTime = loopTime.milliseconds();
                }
                if (!controls.specMode.value()) {
                    transferState = TransferState.RETURN_TO_SAMPLE_MODE;
                    startTime = loopTime.milliseconds();
                }
                if (controls.extend.value()) {
                    arm.pickSpec();
                    arm.openClaw();
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                }
                break;
            case RETURN_TO_SAMPLE_MODE:
                outtake.returnToRetracted();
                intake.extendForSpecMode();
                controls.specMode.set(false);
                //if (loopTime.milliseconds() - startTime >= 1500 && loopTime.milliseconds() - startTime <= 2500 ) {
                    arm.pullBackToGoUp();
                if (loopTime.milliseconds() - startTime >= 600) {
                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                    intake.extendoFullRetract();
                    break;
                }
                break;

            case SPEC_SCORING:
                //outtake.extendToSpecimenHighRackLow();
                //intake.extendForOuttake();
                robot.arm.toScoreSpecimen();

                if (!controls.scoreSpec.value()) {
                    outtake.extendToSpecimenHighRackHigh();
                    transferState = TransferState.SPEC_RETRACTING;
                    startTime = loopTime.milliseconds();
                } else {
                    outtake.extendToSpecimenHighRackLow();
                }
                break;
            case SPEC_RETRACTING:
                outtake.extendToSpecimenHighRackHigh();

                if (loopTime.milliseconds() - startTime <= 800 && loopTime.milliseconds() - startTime >= 500) {
                    arm.openClaw();
                } else if (loopTime.milliseconds() - startTime > 800) {
                    transferState = TransferState.SPEC_IDLE;
                    arm.pickSpec();
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
        WRONG_COLOR("WRONG_COLOR"),
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