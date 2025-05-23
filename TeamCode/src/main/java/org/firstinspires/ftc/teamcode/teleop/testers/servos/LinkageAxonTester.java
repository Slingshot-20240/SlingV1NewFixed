package org.firstinspires.ftc.teamcode.teleop.testers.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@Config
@TeleOp (group = "servo tests")
public class LinkageAxonTester extends OpMode {
    private Robot robot;
    private GamepadMapping controls;
    private Intake intake;

    public static double rservoPos = .5;
    public static double pivotServoPos = .5;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        intake.activeIntake.flipUp();

        robot.arm.readyForTransfer();
        robot.arm.openClaw();
    }

    @Override
    public void loop() {
            intake.rightExtendo.setPosition(rservoPos);
            intake.leftExtendo.setPosition(rservoPos);

            intake.activeIntake.pivotAxon.setPosition(pivotServoPos);
    }
}
