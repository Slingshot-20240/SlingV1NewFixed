//package org.firstinspires.ftc.teamcode.teleop.testers.servos;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
//import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
//
//@Config
//@TeleOp
//public class ClawTester extends OpMode {
//    //private Robot robot;
//    private Arm arm;
//    private GamepadMapping controls;
//    public static double target = 0.0;
//
//    @Override
//    public void init() {
//        controls = new GamepadMapping(gamepad1, gamepad2);
//        this.arm = new Arm(hardwareMap);
//    }
//
//    @Override
//    public void loop() {
//        arm.claw.setPosition(target);
//    }
//}
