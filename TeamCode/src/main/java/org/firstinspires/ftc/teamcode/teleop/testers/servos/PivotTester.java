package org.firstinspires.ftc.teamcode.teleop.testers.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.intake.archived.v4bActive;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
@Config
public class PivotTester extends OpMode {
    //private Robot robot;
    private v4bActive v4b;
    private GamepadMapping controls;
    public static double target = 0.0;


    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        //robot = new Robot(hardwareMap, telemetry, controls);
        this.v4b = new v4bActive(hardwareMap, telemetry, controls);
    }

    @Override
    public void loop() {
        v4b.v4b.setPosition(target);
    }
}
