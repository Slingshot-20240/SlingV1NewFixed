package org.firstinspires.ftc.teamcode.fsm;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorModule;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class ActiveCycle {
    // ROBOT
    // -------
    private final Robot robot;
    private final GamepadMapping controls;
    private ActiveCycle.TransferState transferState;

    // MECHANISMS
    // ---------------
    private final Intake intake;
    private final Outtake outtake;
    private final Arm arm;
    private final ColorSensorModule colorSensor;

    // OTHER
    // ---------
    private final Telemetry telemetry;
    private final ElapsedTime loopTime;
    private double startTime;
    private boolean safeDeposit = false;

    public ActiveCycle(Telemetry telemetry, GamepadMapping controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.outtake = robot.outtake;
        this.arm = robot.arm;
        this.colorSensor = robot.colorSensor;

        this.telemetry = telemetry;
        this.controls = controls;

        transferState = TransferState.BASE_STATE;

        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        startTime = loopTime.milliseconds();
    }

    public void activeIntakeUpdate() {

        telemetry.addData("transfer state", transferState.toString());

        // touch sensor failsafe
        if(outtake.touchSensor.isPressed()) {
            outtake.resetEncoders();
        }

        // base state reset
        if(controls.botToBaseState.value()) {
            controls.botToBaseState.set(false);
            transferState = TransferState.BASE_STATE;
            controls.resetAllControls();
        }

        // arm position farther for deposit
        if (controls.safeDeposit.value()) {
            safeDeposit = true;
        }

        switch (transferState) {

            case BASE_STATE:
                robot.hardwareSoftReset();
                transferState = ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                break;

            case EXTENDO_FULLY_RETRACTED:
                // constantly set motor power
                outtake.returnToRetracted();
                controls.openClaw.set(false);

                // extend, transfer, high & low basket triggers
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

                // intake while retracted (for transfer purposes)
                if (controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.motorRollerOnToIntake();
                } else {
                    intake.activeIntake.motorRollerOff();
                }

                // spec mode!
                if (controls.specMode.value()) {
                    transferState = TransferState.SPEC_MODE;
                    startTime = loopTime.milliseconds();
                }

                // hang (press bot to base state if mess up)
                if (controls.hang.value()) {
                    transferState = TransferState.HANGING;
                    outtake.hang();
                }

                break;
            case EXTENDO_FULLY_EXTENDED:
                controls.resetSpecControls();
                outtake.returnToRetracted();

                // always fully extended
                intake.extendoFullExtend();

                // sample mode: retract trigger
                if (!controls.extend.value() && !controls.specMode.value()) {
                    controls.transfer.set(false);
                    controls.resetOuttakeControls();

                    // set up for transfer
                    intake.extendToTransfer();
                    intake.activeIntake.flipToTransfer();
                    arm.readyForTransfer();
                    arm.openClaw();
                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                // spec mode: retract trigger
                } else if (!controls.extend.value() && controls.specMode.value()) {
                    transferState = TransferState.SPEC_IDLE;
                }

                // intaking triggers (intake, clear or spec)
                if (controls.intakeOnToIntake.locked() || controls.pivotToClear.locked() || controls.pivotToClearSpec.locked()) {
                    transferState = TransferState.INTAKING;
                }

                break;

            case INTAKING:
                outtake.returnToRetracted();

                // retract trigger -> sends to extended state to determine sample/spec mode
                if (!controls.extend.value()) {
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.clearIntake.set(false);
                    controls.intakeOnToIntake.set(false);

                // intake normally
                } else if (controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();

                // to clearing position
                } else if (controls.pivotToClear.locked()) {
                    intake.activeIntake.flipDownToClear();
                    // if clear button pressed, clear
                    if (controls.clearIntake.value()) {
                        intake.activeIntake.motorRollerOnToClear();
                    } else {
                        intake.activeIntake.motorRollerOff();
                    }

                // to spec clearing position (feed samples to other teams)
                } else if (controls.pivotToClearSpec.locked()) {
                    intake.activeIntake.flipDownFull();
                    // TODO: does Souren want this automatic?
                    if (controls.clearIntake.value()) {
                        intake.activeIntake.motorRollerOnToClear();
                    } else {
                        intake.activeIntake.motorRollerOff();
                    }

                // flip up
                } else if (!controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.flipUp();
                    intake.activeIntake.motorRollerOff();
                } else if (!controls.pivotToClear.locked()) {
                    intake.activeIntake.flipUp();
                    intake.activeIntake.motorRollerOff();
                } else if (!controls.pivotToClearSpec.locked()) {
                    intake.activeIntake.flipUp();
                    intake.activeIntake.motorRollerOff();
                }

                // check color sensor
                if (colorSensor.opposingColor()) {
                    intake.activeIntake.clearIntake();
                    transferState = TransferState.WRONG_COLOR;
                    //startTime = loopTime.milliseconds();
                }

                break;

            case WRONG_COLOR:

                // if no longer detects
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

                // transfer trigger
                if(controls.transfer.value()) {
                    intake.activeIntake.transferSample();
                } else {
                    intake.activeIntake.motorRollerOff();
                }

                // close/open claw
                if (controls.openClaw.value()) {
                    arm.closeClaw();
                } else {
                    arm.openClaw();
                }

                // high basket
                if (controls.highBasket.value()) {
                    arm.closeClaw();
                    controls.openClaw.set(false);
                    controls.transfer.set(false);
                    transferState = ActiveCycle.TransferState.HIGH_BASKET;
                    robot.arm.pullBackToGoUp();
                    startTime = loopTime.milliseconds();
                }

                // low basket
                if (controls.lowBasket.value()) {
                    arm.closeClaw();
                    controls.openClaw.set(false);
                    controls.transfer.set(false);
                    transferState = ActiveCycle.TransferState.LOW_BASKET;
                    robot.arm.pullBackToGoUp();
                    startTime = loopTime.milliseconds();
                }

                // extend & reset
                if (controls.extend.value()) {
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetMultipleControls(controls.openClaw, controls.lowBasket, controls.highBasket, controls.transfer);
                }

                // intake while transfering, just in case
                if (controls.intakeOnToIntake.locked() && !controls.transfer.locked()) {
                    intake.activeIntake.motorRollerOnToIntake();
                } else if (controls.intakeOnToIntake.locked() && !controls.transfer.locked()){
                    intake.activeIntake.motorRollerOff();
                }

                break;

            case HIGH_BASKET:
                intake.extendToTransfer();
                outtake.extendToHighBasket();
                intake.activeIntake.motorRollerOff();

                // gives some time for the arm to pull back before going up
                if (loopTime.milliseconds() - startTime >= 300) {
                    if (safeDeposit) {
                        robot.arm.toSafeSampleScore();
                    } else {
                        robot.arm.toScoreSample();
                    }
                }

                // open claw
                if (controls.openClaw.value()) {
                    arm.openClaw();
                }

                // go back down
                if (!controls.highBasket.value()) {
                    transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
                }

                // extend while up (automatically brings slides down)
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
                intake.activeIntake.motorRollerOff();

                // gives some time for the arm to pull back before going up
                if (loopTime.milliseconds() - startTime >= 300) {
                    if (safeDeposit) {
                        robot.arm.toSafeSampleScore();
                    } else {
                        robot.arm.toScoreSample();
                    }
                }

                // open claw
                if (controls.openClaw.value()) {
                    arm.openClaw();
                }

                // go down
                if (!controls.lowBasket.value()) {
                    transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
                }

                // extend trigger
                if (controls.extend.value()) {
                    intake.extendoFullExtend();
                    arm.pullBackToGoUp();
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                }

                break;

            case SLIDES_RETRACTED:
                // reset all control jawns
                controls.resetOuttakeControls();
                controls.resetMultipleControls(controls.transfer, controls.extend, controls.scoreSpec, controls.openClaw, controls.intakeOnToIntake);

                // reset everything to be ready to intake again
                robot.arm.pullBackToGoUp();
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                transferState = ActiveCycle.TransferState.EXTENDO_FULLY_RETRACTED;
                intake.activeIntake.motorRollerOff();

                break;

            case HANGING:

                // go to low pos to hang
                if (!controls.hang.value()) {
                    outtake.moveTicks(OuttakeConstants.SlidePositions.HANGING_LOW.getSlidePos());
                    //transferState = ActiveCycle.TransferState.SLIDES_RETRACTED;
                }

                break;

            case SPEC_MODE:
                // reset + extend
                outtake.returnToRetracted();
                intake.extendForSpecMode();

                // move arm, wait a bit, then go to spec mode
                if (loopTime.milliseconds() - startTime >= 500 && loopTime.milliseconds() - startTime <= 1000 ) {
                    arm.pickSpec();
                } else if (loopTime.milliseconds() - startTime > 1000) {
                    transferState = TransferState.SPEC_IDLE;
                    break;
                }

                break;

            case SPEC_IDLE:
                // reset: ready for spec mode
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                intake.activeIntake.motorRollerOff();
                intake.activeIntake.pivotSpecMode();
                arm.pickSpec();

                // open/close claw
                if (controls.openClaw.value()) {
                    arm.closeClaw();
                } else if (!controls.openClaw.value()) {
                    arm.openClaw();
                }

                // move up to high pos and move arm
                if (controls.scoreSpec.value()) {
                    outtake.extendToSpecimenHighRackLow();
                    transferState = TransferState.SPEC_SCORING;
                    startTime = loopTime.milliseconds();
                }

                // back to sample mode
                if (!controls.specMode.value()) {
                    transferState = TransferState.RETURN_TO_SAMPLE_MODE;
                    startTime = loopTime.milliseconds();
                }

                // extend so we can intake too (state accounts for different retract stuff and brings it back here)
                if (controls.extend.value()) {
                    arm.pickSpec();
                    arm.openClaw();
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                }

                break;
            case RETURN_TO_SAMPLE_MODE:
                // extend again
                outtake.returnToRetracted();
                intake.extendForSpecMode();

                // pull arm back
                controls.specMode.set(false);
                arm.pullBackToGoUp();

                // retract -> ready to sample!
                if (loopTime.milliseconds() - startTime >= 600) {
                    transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                    intake.extendoFullRetract();
                    break;
                }

                break;

            case SPEC_SCORING:
                // arm to pos
                robot.arm.toScoreSpecimen();

                // move up to score
                if (!controls.scoreSpec.value()) {
                    outtake.extendToSpecimenHighRackHigh();
                    transferState = TransferState.SPEC_RETRACTING;
                    startTime = loopTime.milliseconds();
                } else {
                    outtake.extendToSpecimenHighRackLow();
                }

                break;

            case SPEC_RETRACTING:
                // scoring pos
                outtake.extendToSpecimenHighRackHigh();

                // open claw
                if (loopTime.milliseconds() - startTime <= 800 && loopTime.milliseconds() - startTime >= 500) {
                    arm.openClaw();

                // wait a bit, then move back down to spec idle
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