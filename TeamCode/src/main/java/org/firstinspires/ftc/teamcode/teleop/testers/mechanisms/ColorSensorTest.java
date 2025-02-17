package org.firstinspires.ftc.teamcode.teleop.testers.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@Config
@TeleOp
@Disabled
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
        controls.isBlue.update(gamepad1.x);
        if (controls.isBlue.value()) {
            intake.activeIntake.colorSensor.setIsBlue(true);
        } else {
            intake.activeIntake.colorSensor.setIsBlue(false);
        }
        telemetry.addData("Color Sensor Is Blue", intake.activeIntake.colorSensor.getIsBlue());
        telemetry.addData("Is Blue", controls.isBlue.value());
    }

    @Override
    public void loop() {
        controls.update();
        if (controls.intakeOnToIntake.locked()) {
            intake.activeIntake.motorRollerOnToIntake();
        } else {
            intake.activeIntake.motorRollerOff();
        }
        if (intake.activeIntake.colorSensor.checkSample().equals(IntakeConstants.SampleTypes.BLUE) && !intake.activeIntake.colorSensor.isBlue) {
            // add motor to pushOutSample to make it faster
            intake.activeIntake.backRollerServo.setPosition(1);
            intake.activeIntake.motorRollerOnToIntake();
        } else if (intake.activeIntake.colorSensor.checkSample().equals(IntakeConstants.SampleTypes.RED) && intake.activeIntake.colorSensor.isBlue) {
            intake.activeIntake.backRollerServo.setPosition(1);
            intake.activeIntake.motorRollerOnToIntake();
        } else if (intake.activeIntake.colorSensor.checkSample().equals(IntakeConstants.SampleTypes.BLUE) && intake.activeIntake.colorSensor.isBlue
                || intake.activeIntake.colorSensor.checkSample().equals(IntakeConstants.SampleTypes.RED) && !intake.activeIntake.colorSensor.isBlue) {
            pushOut = false;
        }

        telemetry.addData("Sample: ", intake.activeIntake.colorSensor.checkSample());
        telemetry.addData("Color Sensor Is Blue", intake.activeIntake.colorSensor.getIsBlue());
        telemetry.addData("Is Blue", controls.isBlue.value());
        telemetry.addData("loop time", loopTime.milliseconds());
        telemetry.addData("start time", startTime);

//        if (loopTime.milliseconds() - startTime <= 2000) {
//            intake.backRollerServo.setPosition(1);
//            intake.motorRollerOnToIntake();
//        } else {
//            intake.motorRollerOff();
//            intake.backRollerIdle();
//        }

    }
}
