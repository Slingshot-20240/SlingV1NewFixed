package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorI2C.isBlue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.fsm.ActiveCycle;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
public class RoboCampDemo extends OpMode {
    private GamepadMapping controls;
    private ActiveCycle cycle;
    private Robot robot;
    private long previousTime;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        cycle = new ActiveCycle(telemetry, controls, robot);

        robot.outtake.setMotorsToTeleOpMode();

        robot.colorSensorI2C.setIsBlue(isBlue);

        if (controls.botToBaseState.value()) {
            robot.colorSensorI2C.setIsBlue(!isBlue);
            controls.botToBaseState.set(false);
        }

        telemetry.addData("is blue?", isBlue);
    }

    @Override
    public void start() {
        // run once when we start
        robot.hardwareSoftReset();
        robot.intake.extendoFullRetract();

        previousTime = System.currentTimeMillis();
        telemetry.addData("is blue?", isBlue);
    }

    @Override
    public void loop() {
        cycle.activeIntakeUpdate();
        controls.roboCampUpdate();
        robot.drivetrain.update();

        telemetry.addData("is blue?", isBlue);

        long currentTime = System.currentTimeMillis();
        long loopTime = currentTime - previousTime;
        previousTime = currentTime;
        telemetry.addData("Loop Time (ms)", loopTime);
        telemetry.addData("Loop Rate (hz)", 1000 / loopTime);
        telemetry.update();
    }
}
