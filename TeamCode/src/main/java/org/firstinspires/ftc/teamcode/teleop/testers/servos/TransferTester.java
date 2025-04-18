package org.firstinspires.ftc.teamcode.teleop.testers.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp (group = "servo tests")
@Config
public class TransferTester extends OpMode {
    private Robot robot;
    private Arm arm;
    private GamepadMapping controls;
    public static double wristTarget = OuttakeConstants.ArmPositions.RETRACTED.getWristPos();
    public static double armTarget = OuttakeConstants.ArmPositions.RETRACTED.getArmPos();
    public static double pivotPos = IntakeConstants.ActiveIntakeStates.TRANSFER.pivotPos();
    public static double linkagePos = IntakeConstants.ActiveIntakeStates.TRANSFER.rLinkagePos();
    public static boolean runRollers = false;
    public static double slideTarget = 0;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        this.arm = new Arm(hardwareMap);
        robot.intake.extendToTransfer();
        robot.intake.activeIntake.flipToTransfer();
    }

    @Override
    public void loop() {
        arm.armPivot.setPosition(armTarget);
        arm.wrist.setPosition(wristTarget);
        robot.intake.activeIntake.pivotAxon.setPosition(pivotPos);
        robot.intake.rightExtendo.setPosition(linkagePos);
        robot.intake.leftExtendo.setPosition(linkagePos);

        robot.outtake.moveTicks(slideTarget);

        if (runRollers) {
            robot.intake.activeIntake.transferSample();
        } else {
            robot.intake.activeIntake.rollerMotor.setPower(0);
        }
    }
}
