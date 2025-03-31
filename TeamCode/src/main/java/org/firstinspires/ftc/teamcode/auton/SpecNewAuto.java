package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class SpecNewAuto extends LinearOpMode {

    private GamepadMapping controls;
    private Robot robot;
    private Intake intake;
    private Arm arm;

    public double hpX = 40-1.5;
    public double hpY = -71;
    public double scoreY = -26.5;

    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        arm = robot.arm;
        //specimenClaw = robot.specimenClaw;
        robot.intake.extendoFullRetract();
        robot.outtake.resetEncoders();
        robot.intake.activeIntake.pivotAxon.setPosition(.42);
        moveLift(0);
        //outtake.bucketToReadyForTransfer();
        intake.extendoFullRetract();
        arm.closeClaw();

        //robot.hardwareHardReset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(13.25, -64.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preloaded spec
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(0);
                    arm.toScoreSpecimen();
                    //raise slides (small);
                    //flip arm to score
                })
                .lineToConstantHeading(new Vector2d(8,  scoreY-7))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                    moveLift(0);
                })
                .waitSeconds(.5)

                //pickup 1
                .splineToConstantHeading(new Vector2d(26,  -45),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36.5,  -32),Math.toRadians(90))


                .lineToConstantHeading(new Vector2d(36.5,  -13))
                .splineToConstantHeading(new Vector2d(43,  -13),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(42.5,  -13))
                .lineToConstantHeading(new Vector2d(42.5,-55))

                //pickup 2
                .lineToConstantHeading(new Vector2d(42,-13))
                .splineToConstantHeading(new Vector2d(53,  -13),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(53,-55))

                //pickup 3
                .lineToConstantHeading(new Vector2d(47,  -13))
                .splineToConstantHeading(new Vector2d(62,  -13),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(62,-50))

                //HP 1
                .lineToConstantHeading(new Vector2d(hpX,hpY+9))
                .lineToConstantHeading(new Vector2d(hpX-3,hpY))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    arm.toScoreSpecimen();
                    moveLift(0);
                })

                //score 1
                .lineToConstantHeading(new Vector2d(-10,  scoreY-8))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    moveLift(0);
                })
                //HP 2
                .lineToConstantHeading(new Vector2d(hpX,hpY+17))
                .lineToConstantHeading(new Vector2d(hpX,hpY))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    arm.toScoreSpecimen();
                    moveLift(0);
                })
                //score 2
                .lineToConstantHeading(new Vector2d(-6,  scoreY-6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    moveLift(0);
                })

                //HP 3
                .lineToConstantHeading(new Vector2d(hpX,hpY+17))
                .lineToConstantHeading(new Vector2d(hpX,hpY))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    arm.toScoreSpecimen();
                    moveLift(0);
                })
                //score 3
                .lineToConstantHeading(new Vector2d(-2,  scoreY-8))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1000);
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                    arm.pickSpec();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    moveLift(0);
                })
                //HP 4
                .lineToConstantHeading(new Vector2d(hpX,hpY+17))
                .lineToConstantHeading(new Vector2d(hpX,hpY))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    arm.toScoreSpecimen();
                    moveLift(0);
                })
                //score 4
                .lineToConstantHeading(new Vector2d(2,  scoreY-7.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1500);
                    moveExtendo(0.1);
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.openClaw();
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.flipDownFull();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    moveLift(270);
                    arm.readyForTransfer();
                })
                .waitSeconds(0.2)
                //park
                .lineToLinearHeading(new Pose2d(hpX-9,  hpY+24,Math.toRadians(-45)))
                .waitSeconds(0.3)
                //transfer
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.flipToTransfer();
                    intake.extendoFullRetract();
                })
                .waitSeconds(0.1)
                //transfer
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.75);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    arm.closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.95, () -> {
                    arm.toScoreSample();
                    moveLift(1800);

                })
                .lineToLinearHeading(new Pose2d(-53,  -57, Math.toRadians(35)))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    arm.openClaw();

                })
                .forward(5)

                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
            moveLift(0);
            arm.readyForTransfer();
    }

    public void moveLift(int ticks){
        robot.outtake.outtakeSlideLeft.setTargetPosition(ticks);
        robot.outtake.outtakeSlideRight.setTargetPosition(ticks);
        robot.outtake.outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake.outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake.outtakeSlideLeft.setPower(1);
        robot.outtake.outtakeSlideRight.setPower(1);
    }
    public void moveExtendo(double pos){
        robot.intake.leftExtendo.setPosition(pos);
        robot.intake.rightExtendo.setPosition(pos);
    }

}