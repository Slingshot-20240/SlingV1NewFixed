package org.firstinspires.ftc.teamcode.teleop.testers.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@Config
@TeleOp
public class PivotAxonTester extends OpMode {

    private Robot robot;
    private GamepadMapping controls;
    private Intake intake;
    public static double servoPos = .2;
    public static double linkagePos = .5;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        intake.activeIntake.pivotAxon.setPosition(IntakeConstants.ActiveIntakeStates.FULLY_RETRACTED.pivotPos());
        //robot.arm.retract();
        //robot.arm.openClaw();
        intake.extendForOuttake();
    }

    @Override
    public void loop() {
        intake.leftExtendo.setPosition(linkagePos);
        intake.rightExtendo.setPosition(linkagePos);
        intake.activeIntake.pivotAxon.setPosition(servoPos);
    }
}