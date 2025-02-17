package org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure;

import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.RobotMovement.movement_turn;
import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.RobotMovement.movement_x;
import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.RobotMovement.movement_y;
import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.RobotMovement.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.RobotMovement.worldXPosition;
import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.RobotMovement.worldYPosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

import java.util.ArrayList;

public class SamplePathPP extends OpMode {
    ArrayList<CurvePoint> Square = new ArrayList<>();
    private Follower drive;

    @Override
    public void init() {
        drive = new Follower(hardwareMap,telemetry, gamepad1, gamepad2);

        //define localizer starting params
        drive.setStartPos(new Pose2d(0,0,Math.toRadians(0)));

        //DEFINE SQUARE
        Square.add(new CurvePoint(0,0,1,1,20, Math.toRadians(50), 1));
        Square.add(new CurvePoint(40,0,1,1,20, Math.toRadians(50), 1));
        Square.add(new CurvePoint(40,40,1,1,20, Math.toRadians(50), 1));
        Square.add(new CurvePoint(0,40,1,1,20, Math.toRadians(50), 1));
        Square.add(new CurvePoint(0,0,1,1,20, Math.toRadians(50), 1));
    }

    @Override
    public void loop() {
        drive.follow(Square,90);
    }
}
