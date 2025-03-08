package org.firstinspires.ftc.teamcode.teleop.testers.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@Config
@TeleOp
public class ColorSensorTest extends OpMode {
    // to extend, extend out slides a little bit then pivot down
    private Robot robot;
    private GamepadMapping controls;
    private Intake intake;
    private Outtake outtake;
    private DriveTrain dt;

    private ElapsedTime loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double startTime;

    private boolean pushOut = false;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        outtake = robot.outtake;
        dt = robot.drivetrain;
//        intake.extendoFullRetract();
//        intake.flipUp();
        outtake.resetEncoders();
        outtake.returnToRetracted();
        startTime = loopTime.milliseconds();

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {
        controls.update();
        robot.drivetrain.update();
        if (controls.extend.value()) {
            intake.extendoFullExtend();
        } else {
            intake.extendoFullRetract();
        }
        if (controls.intakeOnToIntake.locked()) {
            intake.activeIntake.motorRollerOnToIntake();
            intake.activeIntake.flipDownFull();
        } else if (robot.colorSensorI2C.opposingColor()) {
            intake.activeIntake.clearIntake();
        }

        telemetry.addData("Sample: ", robot.colorSensorI2C.checkSample());
        telemetry.addData("loop time", loopTime.milliseconds());
        telemetry.addData("start time", startTime);

        if (robot.colorSensorI2C.opposingColor()) {
            intake.activeIntake.clearIntake();
        }

//        if (loopTime.milliseconds() - startTime <= 2000) {
//            intake.backRollerServo.setPosition(1);
//            intake.motorRollerOnToIntake();
//        } else {
//            intake.motorRollerOff();
//            intake.backRollerIdle();
//        }

    }
}
