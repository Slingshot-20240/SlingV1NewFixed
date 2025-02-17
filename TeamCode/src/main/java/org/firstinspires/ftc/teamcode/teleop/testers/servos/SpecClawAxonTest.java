package org.firstinspires.ftc.teamcode.teleop.testers.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;

@TeleOp
@Config
public class SpecClawAxonTest extends OpMode {
    public static double servoPos = 0.5;
    private SpecimenClaw specClaw;

    @Override
    public void init() {
        specClaw = new SpecimenClaw(hardwareMap);
    }

    @Override
    public void loop() {
        specClaw.clawServo.setPosition(servoPos);
    }
}
