package org.firstinspires.ftc.teamcode.mechanisms.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorI2C;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

import java.text.NumberFormat;
import java.util.HashMap;
import java.util.Locale;

@TeleOp
@Config

public class VisionTestOpMode extends OpMode {
    Robot robot;
    ColorSensorI2C colorSensor;
    Limelight limelight;
    GamepadMapping controls;

    boolean[] colors = new boolean[3];

    HashMap<String, Boolean> pressed;
    Locale locale;
    NumberFormat nf;
    @Override
    public void init() {

        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        colorSensor = robot.colorSensorI2C;

        limelight = new Limelight(hardwareMap, false, false, false);
        pressed = new HashMap<>();
        pressed.put("a", false);
        pressed.put("b", false);
        pressed.put("x", false);

        locale = Locale.ENGLISH;
        nf = NumberFormat.getNumberInstance(locale);
        nf.setMaximumFractionDigits(2);

    }

    @Override
    public void loop() {
        telemetry.clear();

        /**
         * LIMELIGHT UPDATES
         * colors: [red, blue, ylellow]
         * a: toggle limelight red
         * b: toggle limelight blue
         * x: toggle limelight yellow
         */

        if(gamepad1.a && Boolean.FALSE.equals(pressed.get("a"))){
            colors[0] = !colors[0];
            limelight.setColors(colors);
            pressed.put("a", true);
        }else{
            pressed.put("a", false);
        }
        if(gamepad1.b && Boolean.FALSE.equals(pressed.get("b"))){
            colors[1] = !colors[1];
            limelight.setColors(colors);
            limelight.setColors(colors);
            pressed.put("b", true);

        }else{
            pressed.put("b", false);
        }
        if(gamepad1.x && Boolean.FALSE.equals(pressed.get("x"))){
            colors[2] = !colors[2];
            limelight.setColors(colors);
            limelight.setColors(colors);
            pressed.put("x", true);

        }else if(!gamepad1.x){
            pressed.put("x", false);
        }
        telemetry.addData("limelight (x,y):", String.format("(%s, %s)",
                limelight.location()[0],
                limelight.location()[1]));

        telemetry.addData("limelight (extend, translate):", String.format("(%s, %s",
                nf.format(limelight.getVals()[0]),
                nf.format(limelight.getVals()[1])));

        telemetry.addData("ColorSensor Sample:", colorSensor.checkSample().toString());

        telemetry.addData("ColorSensor colors (r,g,b):",
                            String.format("(%s, %s, %s)",
                                    colorSensor.sensorVals()[0],
                                    colorSensor.sensorVals()[1],
                                    colorSensor.sensorVals()[2]));
        telemetry.addData("Limelight check  (r,b,y):",
                String.format("(%s, %s, %s)",
                        colors[0],
                        colors[1],
                        colors[2]));


        telemetry.update();
    }
}
