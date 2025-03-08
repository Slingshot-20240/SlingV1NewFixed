package org.firstinspires.ftc.teamcode.teleop.testers.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.fsm.ActiveCycle;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
public class PresTele extends OpMode {
    private GamepadMapping controls;
    private ActiveCycle cycle;
    private Robot robot;
    private Intake intake;
    private Outtake outtake;
    private long previousTime;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        cycle = new ActiveCycle(telemetry, controls, robot);

        intake = robot.intake;
        outtake = robot.outtake;

        robot.outtake.setMotorsToTeleOpMode();
        robot.outtake.resetEncoders();

        robot.intake.resetHardware();
        robot.outtake.resetHardware();
    }

    @Override
    public void start() {
        previousTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // already does dt.update();
        cycle.activeIntakeUpdate();
        controls.presModeUpdate();
        telemetry.addData("transferState", cycle.getState().stateName());

        long currentTime = System.currentTimeMillis();
        long loopTime = currentTime - previousTime;
        previousTime = currentTime;
        telemetry.addData("Loop Time (ms)", loopTime);
        telemetry.addData("Loop Rate (hz)", 1000 / loopTime);
        telemetry.update();
    }
}
