package org.firstinspires.ftc.teamcode.teleop.testers.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.fsm.archived.ClawCycle;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
@Disabled
public class ClawTest extends OpMode {
    private Robot robot;
    private GamepadMapping controls;
    private ClawCycle cycle;
    private ElapsedTime loopTime;
    private double startTime;
    private double wristYaw = .1; // min wrist pos

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        cycle = new ClawCycle(telemetry, controls, robot);

        // robot.intake.claw.resetClaw();

        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        startTime = loopTime.milliseconds();
    }

    @Override
    public void loop() {
        controls.update();
        robot.drivetrain.update();

        // double wristSpeed = 0.4;
        // robot.intake.claw.controlWristPos();

        if (controls.extend.value()) {
            robot.intake.extendoFullExtend();
        } else {
            robot.intake.extendoFullRetract();
        }
    }
}
