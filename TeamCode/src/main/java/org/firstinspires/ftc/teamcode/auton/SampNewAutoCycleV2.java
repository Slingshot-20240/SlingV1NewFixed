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
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.specimen.SpecimenClaw;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Config
@Autonomous
public class SampNewAutoCycleV2 extends LinearOpMode {

    private GamepadMapping controls;
    private Robot robot;
    private static IntakeConstants.ActiveIntakeStates activeIntakeStates;
    private Intake intake;
    private Outtake outtake;
    private SpecimenClaw specimenClaw;

    public static double scorePosX = -53;
    public static double scorePosY = -53;

    public static double pick1Angle = 87;
    public static double pick2Angle = 110;
    public static double pick3Angle = 143;

    //TODO: when implementing extendo positions, use these to make it configurable
    public static double pick1Ext = 0;
    public static double pick2Ext = 0;
    public static double pick3Ext = 0;


    Pose2d startPose = new Pose2d(-37.5, -63.5, Math.toRadians(0));
    public double robotX = -37.5;
    public double robotY = -63.5;

    enum State {
        samp4State,   //does 4 sample, goes to pickup
        pickUpState,   // goes to pickup position, goes to limelight state
        limelightState, //gets us the offset of the largest blob then drives to it, intake, goes to sensor state
        sensorState, //checks the sensor. If its bad, it goes into spit state, if its good, it goes into score state
        spitState,   // go back to limelight pos, then spit, then go back to limelight state
        scoreState, //transfer and score, goes to pickup
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, telemetry, controls);
        intake = robot.intake;
        outtake = robot.outtake;
        //specimenClaw = robot.specimenClaw;
        robot.outtake.resetEncoders();
        moveLift(0);
        //outtake.bucketToReadyForTransfer();
        intake.extendoFullRetract();
        intake.activeIntake.flipUp();
        specimenClaw.closeClaw();

        //robot.hardwareHardReset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        drive.setPoseEstimate(startPose);

        TrajectorySequence sample4Path = drive.trajectorySequenceBuilder(startPose)
                //preload
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //raise slides
                    //flipArm
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //openClaw
                    //pivot out; extendo small
                })
                .waitSeconds(0.15)

                //pickUp1
                .lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(pick1Angle)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Extendo More
                })
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //stopIntake
                    //pivot up; extendo in
                })

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    //Transfer

                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //slides up
                    //arm flip
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //openClaw
                    //pivot out; extendo small
                    //intake Run
                })
                .waitSeconds(0.15)

                //pickUp2
                .lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(pick2Angle)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Extendo More
                })
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //stopIntake
                    //pivot up; extendo in
                })

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    //Transfer

                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //slides up
                    //arm flip
                })
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //openClaw
                    //pivot out; extendo small
                })
                .waitSeconds(0.15)

                //pickUp3
                .lineToLinearHeading(new Pose2d(-47, -43, Math.toRadians(pick3Angle)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Extendo More
                })
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //stopIntake
                    //pivot up; extendo in
                })

                //score
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    //Transfer

                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //slides up
                    //arm flip
                })

                //score
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //openClaw
                    //pivot out; extendo small
                })
                .waitSeconds(0.15)
                .forward(4)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.samp4State;
        drive.followTrajectorySequenceAsync(sample4Path);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case samp4State:
                    if (!drive.isBusy()) {
                        currentState = State.pickUpState;
                        pickUpPath(drive, drive.getPoseEstimate());
                    }
                    break;
                case limelightState:
                    if (!drive.isBusy()) {
                        //Check limelight for offsets
                        // use offsets for path
                        limeLightPath(drive, drive.getPoseEstimate());
                        currentState = State.sensorState;

                    }
                    break;
                case sensorState:
                    if (!drive.isBusy()) {
                        //if statement that checks if correct color or not
                        //if yes:
                        currentState = State.scoreState;
                        scorePath(drive, drive.getPoseEstimate());
                        //else
                        currentState = State.spitState;
                        spitPath(drive, drive.getPoseEstimate());
                    }
                    break;
                case spitState:
                    if (!drive.isBusy()) {
                        //drive back to pickup pos jawn then spit
                        currentState = State.limelightState;
                    }
                    break;
                case scoreState:
                    if (!drive.isBusy()) {
                        //drive back to pickup pos jawn then spit
                        currentState = State.pickUpState;
                        pickUpPath(drive, drive.getPoseEstimate());                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            robotX = poseEstimate.getX();
            robotY = poseEstimate.getY();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void moveLift(int ticks){
        robot.outtake.outtakeSlideLeft.setTargetPosition(ticks);
        robot.outtake.outtakeSlideRight.setTargetPosition(ticks);
        robot.outtake.outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake.outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtake.outtakeSlideLeft.setPower(1);
        robot.outtake.outtakeSlideRight.setPower(1);
    }

    public void pickUpPath(SampleMecanumDrive drive, Pose2d poseEstimate){
        TrajectorySequence Traj = drive.trajectorySequenceBuilder(poseEstimate)
                .forward(10)
                .build();

        drive.followTrajectorySequenceAsync(Traj);
    }

    public void limeLightPath(SampleMecanumDrive drive, Pose2d poseEstimate){
        TrajectorySequence Traj = drive.trajectorySequenceBuilder(poseEstimate)
                //run intake, extend extendo, lower pivot
                .lineToConstantHeading(new Vector2d(robotX, robotY)) //TODO: ADD OFFSET TO Y
                .forward(10)
                .build();

        drive.followTrajectorySequenceAsync(Traj);
    }

    public void spitPath(SampleMecanumDrive drive, Pose2d poseEstimate){
        TrajectorySequence Traj = drive.trajectorySequenceBuilder(poseEstimate)
                .forward(10)
                .build();

        drive.followTrajectorySequenceAsync(Traj);
    }

    public void scorePath(SampleMecanumDrive drive, Pose2d poseEstimate){
        TrajectorySequence Traj = drive.trajectorySequenceBuilder(poseEstimate)
                .forward(10)
                .build();

        drive.followTrajectorySequenceAsync(Traj);
    }

    public void moveExtendo(double pos){
        robot.intake.leftExtendo.setPosition(pos);
        robot.intake.rightExtendo.setPosition(pos);
    }

}