package org.firstinspires.ftc.teamcode.teleop.testers.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.fsm.ActiveCycle;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
public class PresTele extends OpMode {
    private GamepadMapping controls;
    private ActiveCycle cycle;
    private Robot robot;
    private Intake intake;
    private Outtake outtake;
    //private SpecimenClaw specimenClaw;
    private double startTime;
    private TransferState transferState;
    private ElapsedTime loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        cycle = new ActiveCycle(telemetry, controls, robot);

        intake = robot.intake;
        outtake = robot.outtake;
        //specimenClaw = robot.specimenClaw;

        robot.outtake.setMotorsToTeleOpMode();
        robot.outtake.resetEncoders();

        robot.intake.resetHardware();
        robot.outtake.resetHardware();
        robot.arm.resetHardware();

        transferState = TransferState.BASE_STATE;

        double startTime = loopTime.milliseconds();
    }

    @Override
    public void loop() {
        controls.presModeUpdate();

        switch (transferState) {
            case BASE_STATE:
                robot.hardwareSoftReset();
                transferState = TransferState.EXTENDO_FULLY_RETRACTED;
                break;
            case EXTENDO_FULLY_RETRACTED:
                // have to constantly set power of slide motors back
                outtake.returnToRetracted();
                //intake.extendoFullRetract();
                controls.openClaw.set(false);
                if (controls.extend.value()) {
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                } else if (controls.transfer.locked()) {
                    transferState = TransferState.TRANSFERING;
                } else if (controls.highBasket.value()) {
                    transferState = TransferState.HIGH_BASKET;
                    robot.arm.toTransfering();
                    startTime = loopTime.milliseconds();
                } else if (controls.lowBasket.value()) {
                    transferState = TransferState.LOW_BASKET;
                    robot.arm.toTransfering();
                    startTime = loopTime.milliseconds();
                }
                if (controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.motorRollerOnToIntake();
                } else {
                    intake.activeIntake.motorRollerOff();
                }
            case EXTENDO_FULLY_EXTENDED:
                controls.resetSpecControls();
                outtake.returnToRetracted();
                intake.extendoFullExtend();
                if (!controls.extend.value()) {
                    intake.extendForOuttake();
                    intake.activeIntake.flipToTransfer();
                    controls.transfer.set(false);
                    controls.resetOuttakeControls();
                    robot.arm.retract();
                    robot.arm.openClaw();
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
                if (!controls.transfer.locked()) {
                    intake.activeIntake.transferOff();
//                    transferState = TransferState.TRANSFER_CLOSE;
//                    startTime = loopTime.milliseconds();
                } else {
                    intake.activeIntake.transferSample();
                }
                if (controls.openClaw.value()) {
                    robot.arm.closeClaw();
                } else {
                    robot.arm.openClaw();
                }
                if (controls.highBasket.value()) {
                    controls.openClaw.set(false);
                    robot.arm.closeClaw();
                    transferState = TransferState.HIGH_BASKET;
                    robot.arm.toTransfering();
                    startTime = loopTime.milliseconds();
                }
                if (controls.lowBasket.value()) {
                    controls.openClaw.set(false);
                    robot.arm.closeClaw();
                    transferState = TransferState.LOW_BASKET;
                    robot.arm.toTransfering();
                    startTime = loopTime.milliseconds();
                }

                break;
            case HIGH_BASKET:
                intake.extendForOuttake();
                outtake.extendToHighBasket();
                if (loopTime.milliseconds() - startTime >= 300) {
                    robot.arm.toScoreSpecimen();
                }

                if (controls.openClaw.value()) {
                    robot.arm.openClaw();
                }
                if (!controls.highBasket.value()) {
                    transferState = TransferState.SLIDES_RETRACTED;
                }
                if (controls.extend.value()) {
                    intake.extendoFullExtend();
                    robot.arm.toTransfering();
                    transferState = TransferState.EXTENDO_FULLY_EXTENDED;
                    controls.resetOuttakeControls();
                }
                break;
            case SLIDES_RETRACTED:
                controls.resetOuttakeControls();
                controls.resetMultipleControls(controls.transfer, controls.extend, controls.scoreSpec, controls.openClaw);
                // could also do to base state
                robot.arm.toTransfering();
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                transferState = TransferState.EXTENDO_FULLY_RETRACTED;
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
            LOW_BASKET("LOW_BASKET");

            private String state;

            TransferState(String stateName) {
                state = stateName;
            }

            public String stateName() {
                return state;
            }
        }
}
