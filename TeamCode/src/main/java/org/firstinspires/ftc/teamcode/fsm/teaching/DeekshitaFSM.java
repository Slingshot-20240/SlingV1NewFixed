package org.firstinspires.ftc.teamcode.fsm.teaching;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class DeekshitaFSM {
    States state = States.BASE_STATE;
    GamepadMapping controls;

    Robot robot;
    Outtake outtake;
    Intake intake;
    Arm arm;

    ElapsedTime loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double startTime;


    public DeekshitaFSM(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);

        startTime = loopTime.milliseconds();
    }

    public void update() {

        // if base state button pressed
            // reset bot
            // set state back to base state


        switch(state) {

            case BASE_STATE:
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                intake.activeIntake.flipToTransfer();
                arm.openClaw();
                arm.readyForTransfer();

                if(controls.extend.value()) {
                    state = States.FULLY_EXTENDED;
                }

                if(controls.highBasket.value()) {
                    state = States.HIGH_BASKET;
                }

                if (controls.transfer.locked()) {
                    state = States.TRANSFER;
                }

                break;

            case HIGH_BASKET:
                outtake.returnToRetracted();
                intake.extendoFullRetract();
                arm.toScoreSample();

                if (controls.openClaw.value()) {
                    arm.openClaw();
                }

                if (!controls.highBasket.value()) {
                    state = States.BASE_STATE;
                }

                break;

            case INTAKE:
                outtake.returnToRetracted();
                intake.extendoFullExtend();

                if (controls.intakeOnToIntake.locked()) {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                }

                if (controls.pivotToClear.locked()) {
                    intake.activeIntake.flipDownToClear();
                    if (controls.clearIntake.value()) {
                        intake.activeIntake.motorRollerOnToClear();
                    } else {
                        intake.activeIntake.motorRollerOff();
                    }
                }

                if (!controls.intakeOnToIntake.value() && !controls.pivotToClear.value()) {
                    intake.activeIntake.flipUp();
                    intake.activeIntake.motorRollerOff();
                    controls.clearIntake.set(false);
                }

                if (!controls.extend.value()) {
                    state = States.BASE_STATE;
                }

                break;

            case FULLY_EXTENDED:
                outtake.returnToRetracted();
                intake.extendoFullExtend();

                if (controls.intakeOnToIntake.value() || controls.pivotToClear.value()) {
                    state = States.INTAKE;
                }

                if (!controls.extend.value()) {
                    state = States.BASE_STATE;
                }

                break;

            case TRANSFER:
                outtake.returnToRetracted();
                intake.activeIntake.flipToTransfer();
                intake.activeIntake.transferSample();

                if (!controls.transfer.locked()) {
                    state = States.BASE_STATE;
                }

                break;

            case PICKUP:
                intake.extendoFullRetract();
                outtake.returnToRetracted();
                arm.pickSpec();
                if (controls.openClaw.value()){
                    arm.closeClaw();
                } else {
                    arm.openClaw();
                }
                if (controls.scoreSpec.value()){
                    state = States.GO_TO_SCORE;
                }
                break;
            case GO_TO_SCORE:
                outtake.returnToRetracted();
                arm.toScoreSpecimen();
                if (!controls.scoreSpec.value()){
                    state = States.SCORED;
                    startTime = loopTime.milliseconds();
                }
                break;

            case SCORED:
                outtake.extendToSpecimenHighRackHigh();
                if (loopTime.milliseconds() - startTime >= 500 && loopTime.milliseconds() - startTime <=1000){
                    arm.openClaw();
                } else if (loopTime.milliseconds() - startTime > 1000){
                    state = States.BASE_STATE;
                    break;
                }
            break;
        }
    }

    public enum States {
        INTAKE("INTAKE"),
        BASE_STATE("BASE STATE"),
        HIGH_BASKET("HIGH BASKET"),
        FULLY_EXTENDED("FULLY EXTENDED"),
        PICKUP("PICKUP"),
        GO_TO_SCORE("GO_TO_SCORE"),
        SCORED("SCORED"),
        TRANSFER("TRANSFER");

        String name;

        States(String name) {
            this.name = name;
        }

        public String getName() {
            return this.name;
        }

    }
}
