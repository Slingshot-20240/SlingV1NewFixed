package org.firstinspires.ftc.teamcode.teleop.testers.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
@Config
public class WristTester extends OpMode {
    private Robot robot;
    private Arm arm;
    private GamepadMapping controls;
    public static double wristTarget = 0.0;
    public static double armTarget = 0.0;
    public static double pivotPos = IntakeConstants.ActiveIntakeStates.TRANSFER.pivotPos();
    public static double linkagePos = IntakeConstants.ActiveIntakeStates.OUTTAKING.rLinkagePos();
    public static boolean runRollers = false;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        this.arm = new Arm(hardwareMap);
        robot.intake.extendForOuttake();
        robot.intake.activeIntake.flipToTransfer();
    }

    @Override
    public void loop() {
        arm.armPivot.setPosition(armTarget);
        arm.wrist.setPosition(wristTarget);
        robot.intake.activeIntake.pivotAxon.setPosition(pivotPos);
        robot.intake.rightExtendo.setPosition(linkagePos);
        robot.intake.leftExtendo.setPosition(linkagePos);

        if (runRollers) {
            robot.intake.activeIntake.transferSample();
        } else {
            robot.intake.activeIntake.rollerMotor.setPower(0);
        }
    }
}
