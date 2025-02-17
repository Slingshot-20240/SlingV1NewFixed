package org.firstinspires.ftc.teamcode.teleop.testers.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
public class IntakeMotorTester extends OpMode {
    private Robot robot;
    private GamepadMapping controls;
    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
    }

    @Override
    public void loop() {
        controls.update();
        if (controls.intakeOnToIntake.locked()) {
            robot.intake.activeIntake.flipDownFull();
            robot.intake.activeIntake.motorRollerOnToIntake();
        } else {
            robot.intake.activeIntake.motorRollerOff();
            robot.intake.activeIntake.flipUp();
        }
    }
}
