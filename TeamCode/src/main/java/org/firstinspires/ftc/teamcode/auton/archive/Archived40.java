package org.firstinspires.ftc.teamcode.auton.archive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class Archived40 extends LinearOpMode {

    private GamepadMapping controls;
    private Robot robot;
    private static IntakeConstants.ActiveIntakeStates activeIntakeStates;
    private Intake intake;
    private Outtake outtake;
    private SpecimenClaw specimenClaw;
    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        outtake = robot.outtake;
        specimenClaw = robot.specimenClaw;
        robot.outtake.resetEncoders();
        moveLift(0);
        outtake.bucketToReadyForTransfer();
        intake.extendoFullRetract();
        intake.activeIntake.flipUp();
        specimenClaw.closeClaw();

        //robot.hardwareHardReset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)


                //preloaded spec
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1265);
                })
                .waitSeconds(0.5)
                .back(33)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(700);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    //open claw (preloaded spec)
                    specimenClaw.openClaw();
                })
                .waitSeconds(.5)

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    moveLift(0);
                })

                //go2other3samples
                //first sample
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.extendoFullExtend();
                    intake.activeIntake.motorRollerOnToIntake();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    intake.activeIntake.flipDownFull();
                })
                .splineToLinearHeading(new Pose2d(14,-44,Math.toRadians(34)), Math.toRadians(30))
                .waitSeconds(0.1)
                .forward(14)
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(30,-50,Math.toRadians(-40)))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.transferSample();
                })
                .waitSeconds(0.3)

                //second sample
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(33,-45,Math.toRadians(41)), Math.toRadians(0))
                .waitSeconds(0.1)
                .forward(8)
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(30,-50,Math.toRadians(-40)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.transferSample();
                })


                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.motorRollerOff();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.activeIntake.flipToTransfer();
                    intake.extendoFullRetract();
                })

                //go back to hp
                .lineToLinearHeading(new Pose2d(40,-54,Math.toRadians(90)))
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //close claw
                    specimenClaw.closeClaw();
                })
                .waitSeconds(0.5)


                //go to box
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1310);
                })
                .lineToSplineHeading(new Pose2d(8,-38,Math.toRadians(270)))

                .setReversed(true)
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(700);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    //open claw (spec)
                    specimenClaw.openClaw();
                })
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    moveLift(0);
                })
                .setReversed(false)
                .forward(6)

                //go back to hp
                .lineToLinearHeading(new Pose2d(40,-54,Math.toRadians(90)))
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //close claw (intake spec 3)
                    specimenClaw.closeClaw();
                })
                .waitSeconds(0.5)

                //go back to box
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1335);
                })
                .lineToSplineHeading(new Pose2d(5,-38,Math.toRadians(270)))
                .setReversed(true)
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(700);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    //open claw (spec)
                    specimenClaw.openClaw();
                })
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    moveLift(0);
                })
                .setReversed(false)
                .forward(6)



                //go back to hp
                .lineToLinearHeading(new Pose2d(40,-54,Math.toRadians(90)))
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //close claw
                    specimenClaw.closeClaw();
                })
                .waitSeconds(0.5)

                //go back to box
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1335);
                })
                .lineToSplineHeading(new Pose2d(2,-38,Math.toRadians(270)))
                .setReversed(true)
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(700);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    //open claw (spec)
                    specimenClaw.openClaw();
                })
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    moveLift(0);
                })
                .setReversed(false)
                .forward(6)


                //potential last spec :_D
//                                        //go back to box
//                                        .lineToSplineHeading(new Pose2d(4,-36,Math.toRadians(270)))
//                                        .setReversed(true)
//                                        .back(6)
//                                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                            //tiny slides raise
//                                        })
//                                        .waitSeconds(.5)
//                                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                            //open claw (preloaded spec)
//                                        })








                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }

    public void moveLift(int ticks){
        robot.outtake.outtakeSlideLeft.setTargetPosition(ticks);
        robot.outtake.outtakeSlideRight.setTargetPosition(ticks);
        robot.outtake.outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake.outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake.outtakeSlideLeft.setPower(1);
        robot.outtake.outtakeSlideRight.setPower(1);
    }

}