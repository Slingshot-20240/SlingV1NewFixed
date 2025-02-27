package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class SampNewAuto extends LinearOpMode {

    private GamepadMapping controls;
    private Robot robot;
    private Intake intake;
    private Arm arm;

    public double scorePosX = -55.5; // TODO: tune
    public double scorePosY = -55.5;

    public double pick1Angle = 40;
    public double pick2Angle = 55;

    //TODO: when implementing extendo positions, use these to make it configurable
    public double pick1Ext = 0;
    public double pick2Ext = 0;
    public double pick3Ext = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        arm = robot.arm;
        //specimenClaw = robot.specimenClaw;
        robot.outtake.resetEncoders();
        moveLift(0);
        //outtake.bucketToReadyForTransfer();
        intake.extendoFullRetract();
        intake.activeIntake.flipUp();
        arm.closeClaw();

        //robot.hardwareHardReset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-37.5, -63.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preload
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(2000);
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.retract();
                    moveLift(0);
                    moveExtendo(.1);
                    //intake.activeIntake.flipDownFull();
                })

                //pickUp1
//                .turn(Math.toRadians(pick1Angle))
                .lineToLinearHeading(new Pose2d(-46,-54, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.motorRollerOnToIntake();
                    intake.activeIntake.flipDownFull();
                })
                .forward(15)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.extendForOuttake();
                })


                //score
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.35);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    arm.closeClaw();

                })
                .turn(Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    moveLift(2000);
                    intake.activeIntake.transferOff();
                    arm.toScoreSample();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.retract();
                    moveLift(0);
                    moveExtendo(IntakeConstants.ActiveIntakeStates.OUTTAKING.rLinkagePos());
                })

                //pickUp2
                .lineToLinearHeading(new Pose2d(-55, -54, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .forward(15)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.extendForOuttake();
                })

                //score
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.35);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    arm.closeClaw();
                })
                .turn(Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.transferOff();
                    moveLift(2000);
                    arm.toScoreSample();
                })
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.retract();
                    moveLift(0);
                    moveExtendo(IntakeConstants.ActiveIntakeStates.OUTTAKING.rLinkagePos());
                })

                //pickUp3
                .lineToLinearHeading(new Pose2d(-42, -43, Math.toRadians(170)))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                    intake.extendoFullExtend();
                })
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    intake.extendForOuttake();
                })

                //score
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    intake.activeIntake.rollerMotor.setPower(0.35);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    arm.closeClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.activeIntake.transferOff();
                    moveLift(2000);
                    arm.toScoreSample();
                })
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    arm.openClaw();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.retract();
                    moveLift(0);
                })
                .forward(10)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
            moveLift(0);

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