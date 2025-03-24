package org.firstinspires.ftc.teamcode.teleop.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorI2C;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

@TeleOp
@Config

public class ColorSensorCalibRatio extends OpMode {
    public static double redThresh = 1.25;
    public static double blueThresh = 2.25;
    public static double yellowThresh = 1;

    private Telemetry dashboardTelemetry;

    Robot robot;
    GamepadMapping controls;
    ColorSensorI2C sensor;


    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        dashboardTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        sensor = new ColorSensorI2C(hardwareMap);


    }

    @Override
    public void loop() {
        dashboardTelemetry.addData("redVal", sensor.sensor.red());
        dashboardTelemetry.addData("blueVal", sensor.sensor.blue());
        dashboardTelemetry.addData("greenVal", sensor.sensor.green());

        dashboardTelemetry.addData("check sample", sensor.checkSample().toString());

        sensor.changeThres(redThresh, blueThresh, yellowThresh);

        dashboardTelemetry.update();
    }
}
