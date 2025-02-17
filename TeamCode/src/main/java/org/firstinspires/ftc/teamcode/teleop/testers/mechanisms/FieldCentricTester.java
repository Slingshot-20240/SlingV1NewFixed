package org.firstinspires.ftc.teamcode.teleop.testers.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
@Disabled
public class FieldCentricTester extends OpMode {
    // ftc dashboard: 192.168.43.1:8080/dash
    private double drive;
    private double strafe;
    private double turn;
    private double currentAngle;

    private GamepadMapping controls;

    private Robot robot;
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, controls);
        telemetry.addData("currentAngle: ", Math.toDegrees(robot.drivetrain.getHeading()));
        currentAngle = robot.drivetrain.getHeading();
        controls = new GamepadMapping(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        telemetry.clearAll();
        telemetry.addData("currentAngle: ", Math.toDegrees(robot.drivetrain.getHeading()));

        currentAngle = robot.drivetrain.getHeading();

        controls.update();
        robot.drivetrain.update();
    }
}
