package org.firstinspires.ftc.teamcode.teleop.testers.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@Config
@TeleOp (group = "servo tests")
public class ArmTester extends OpMode {
    private Robot robot;
    private Arm arm;
    private GamepadMapping controls;
    public static double armTarget = 0.5;
    public static double wristTarget = 0.7;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        this.arm = new Arm(hardwareMap);
        robot.intake.activeIntake.pivotAxon.setPosition(IntakeConstants.ActiveIntakeStates.SPEC_MODE.pivotPos());
    }

    @Override
    public void loop() {
        arm.wrist.setPosition(wristTarget);
        arm.armPivot.setPosition(armTarget);
    }
}
