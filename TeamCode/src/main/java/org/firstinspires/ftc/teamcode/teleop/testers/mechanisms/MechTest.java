package org.firstinspires.ftc.teamcode.teleop.testers.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.fsm.ActiveCycle;
import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
@Config
@Disabled
public class MechTest extends OpMode {
    private SpecimenClaw specClaw;
    private GamepadMapping controls;
    private Robot robot;
    public static boolean clawOnly = true;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        specClaw = robot.specimenClaw;
    }

    @Override
    public void loop() {
        controls.update();
        if(controls.openClaw.value() && clawOnly) {
            specClaw.openClaw();
        } else if(!controls.openClaw.value() && clawOnly) {
            specClaw.closeClaw();
        }

        if(controls.scoreSpec.value() && !clawOnly) {
            specClaw.closeClaw();
            robot.outtake.extendToSpecimenHighRack();
        } else if (!controls.scoreSpec.value() && !clawOnly) {
            robot.outtake.returnToRetracted();
            // may need to have elapsed time here
            specClaw.openClaw();
        }
    }
}
