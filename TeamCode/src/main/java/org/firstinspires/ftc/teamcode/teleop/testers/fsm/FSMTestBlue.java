package org.firstinspires.ftc.teamcode.teleop.testers.fsm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.fsm.ActiveCycle;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
public class FSMTestBlue extends OpMode {
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
        robot.outtake.resetEncoders();

        robot.intake.resetHardware();
        robot.outtake.resetHardware();

        robot.colorSensorI2C.setIsBlue(true);
    }

    @Override
    public void start() {
        previousTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        cycle.activeIntakeUpdate();
        controls.update();
        robot.drivetrain.update();

        telemetry.addData("transferState", cycle.getState().stateName());
        telemetry.addData("is blue?", robot.colorSensorI2C.isBlue);

        long currentTime = System.currentTimeMillis();
        long loopTime = currentTime - previousTime;
        previousTime = currentTime;
        telemetry.addData("Loop Time (ms)", loopTime);
        telemetry.addData("Loop Rate (hz)", 1000 / loopTime);
        telemetry.update();
    }
}
