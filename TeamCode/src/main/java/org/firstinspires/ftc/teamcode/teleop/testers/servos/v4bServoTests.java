package org.firstinspires.ftc.teamcode.teleop.testers.servos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
@Disabled
public class v4bServoTests extends OpMode {
    private Robot robot;
    private GamepadMapping controls;

    public static double v4bPos = 0; // axon max

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        robot.intake.claw.clawServo.setPosition(0);
    }

    @Override
    public void loop() {
        robot.intake.claw.v4b.setPosition(v4bPos);
    }
}
