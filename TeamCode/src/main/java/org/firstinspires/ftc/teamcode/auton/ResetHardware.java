package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@Autonomous
// this class is for when we cannot/don't run auton. We will run this class to be ready for teleop since we can not move in the transition from auton to teleop
public class ResetHardware extends LinearOpMode {
    private Robot robot;
    private GamepadMapping controls;
    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);

        waitForStart();

        robot.hardwareHardReset();
    }
}
