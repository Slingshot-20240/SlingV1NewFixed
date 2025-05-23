package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorI2C.isBlue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.vision.ColorSensor.ColorSensorI2C;
import org.firstinspires.ftc.teamcode.mechanisms.vision.Limelight;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//TODO: merge and uncomment below
//import org.firstinspires.ftc.teamcode.mechanisms.vision.Limelight;
@Autonomous
public class SampleCyclerNewThingy extends LinearOpMode {
    ElapsedTime limeLightTimer = new ElapsedTime();
    ElapsedTime runTime = new ElapsedTime();
    Pose2d poseEstimate;
    State currentState = State.IDLE;
    SampleMecanumDrive drive;
    private int sample = 0;
    private double cord1 = 0;
    private double cord2 = 0;
    private double cord3 = 0;
    private int tryCount = 0;

    //mechanisms
    private GamepadMapping controls;
    private Robot robot;
    private Intake intake;
    private Arm arm;
    private ColorSensorI2C colorSensor;
//    private Limelight limelight;


    //tunable pos
    public double scorePosX = -54;
    public double scorePosY = -54;

    // outtake
    public int slidePos = 300;

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

        drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-38.5, -63.5, Math.toRadians(0));


        drive.setPoseEstimate(startPose);
        colorSensor = robot.colorSensorI2C;
        colorSensor.setIsBlue(true);
//        limelight = new Limelight(hardwareMap, false, false, true);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preload
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift(1700);
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(scorePosX-4, scorePosY-1, Math.toRadians(74)))
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.openClaw();
                    moveExtendo(.1);
                    intake.activeIntake.motorRollerOnToIntake();
                    intake.activeIntake.flipDownFull();
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    arm.readyForTransfer();
                    moveLift(slidePos);
                    //intake.activeIntake.flipDownFull();
                })

                //pickUp1
                .lineToLinearHeading(new Pose2d(-51,-47.5, Math.toRadians(75)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.extendToTransfer();
                    moveLift(slidePos);
                })
//                .waitSeconds(.1)
                .lineToLinearHeading(new Pose2d(scorePosX+1, scorePosY+1, Math.toRadians(72)))
                .lineToLinearHeading(new Pose2d(scorePosX-6, scorePosY+1, Math.toRadians(72)))


                //score
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.rollerMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.closeClaw();

                })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    arm.pullBackToGoUp();
//                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(1700);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    arm.openClaw();
                    moveExtendo(.1);
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(slidePos);
//                    moveExtendo(IntakeConstants.ActiveIntakeStates.OUTTAKING.rLinkagePos());
                })

                //pickUp2
                .lineToLinearHeading(new Pose2d(-56.5, -45, Math.toRadians(98)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveExtendo(0.1);
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.extendToTransfer();
                    moveLift(slidePos);
                })
                //.waitSeconds(.2)
                .lineToLinearHeading(new Pose2d(scorePosX+1, scorePosY+1, Math.toRadians(73)))
                .lineToLinearHeading(new Pose2d(scorePosX-4.5, scorePosY-1, Math.toRadians(73)))

                //score
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.rollerMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    intake.activeIntake.rollerMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3 , () -> {
                    arm.closeClaw();

                })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    arm.pullBackToGoUp();
//                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(1700);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    arm.openClaw();
                    moveExtendo(.1);
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.readyForTransfer();
                    moveLift(slidePos);
                    //moveExtendo(IntakeConstants.ActiveIntakeStates.TRANSFER.rLinkagePos());
                })

                //pickUp3
//                .lineToLinearHeading(new Pose2d(-36, -44.75, Math.toRadians(170)))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.flipDownFull();
                    intake.activeIntake.motorRollerOnToIntake();
                    moveExtendo(0.1);
                })
                .lineToLinearHeading(new Pose2d(-43, -31, Math.toRadians(173)))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.activeIntake.flipToTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.extendToTransfer();
                    moveLift(slidePos);
                })
                //.waitSeconds(0.6)
                //.lineToLinearHeading(new Pose2d(scorePosX+1, scorePosY+1, Math.toRadians(45)))

                .lineToLinearHeading(new Pose2d(scorePosX+2, scorePosY+1.5, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(scorePosX-0.5, scorePosY-0.5, Math.toRadians(45)))

                //score
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.activeIntake.rollerMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    arm.closeClaw();

                })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    arm.pullBackToGoUp();
//                })
                //.waitSeconds(.1)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(1700);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                //.waitSeconds(.5)
                .waitSeconds(1.5)
                .build();

        while(opModeInInit() && !isStopRequested()){
            //TODO: add controller inputs here for alliance color
            //TODO: add stuff that changes what color we are sensing for in our limelight
            controls.update();
            if (controls.botToBaseState.value()) {
                colorSensor.setIsBlue(!isBlue);
//                limelight.setColors(false, false, true);
                controls.botToBaseState.set(false);
            }

            cord1 -= gamepad2.left_stick_y*0.002;
            cord2 -= gamepad2.right_stick_y*0.002;
            cord3 += (gamepad2.right_trigger- gamepad2.left_trigger)*0.002;
            telemetry.addData("isBlue", isBlue);
            telemetry.addData("cord 5", cord1);
            telemetry.addData("cord 6", cord2);
            telemetry.addData("cord 7", cord3);
            telemetry.update();
        }
        waitForStart();
        runTime.reset();

        if (isStopRequested()) return;

        currentState = State.initialSamplesState;
        drive.followTrajectorySequenceAsync(trajSeq);
        sample = 4;

        while (opModeIsActive() && !isStopRequested()) {

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case initialSamplesState:
                case scoreState:
                    if(!drive.isBusy()) {
                        if (sample == 6) {
                            arm.openClaw();
                            currentState = State.parkState;
                            parkPath(poseEstimate);
                        } else {
                            arm.openClaw();
                            sample++;
                            currentState = State.pickupState;
                            pickUpPath(poseEstimate);
                        }
                    }
                    break;
                case pickupState:
                case spitState:
                    if (!drive.isBusy()) {
                        currentState = State.limelightState;
                        limeLightTimer.reset();
                    }
                    break;
                case limelightState:
                    moveExtendo(0.13);
                    intake.activeIntake.flipDownToClear();
                    intake.activeIntake.rollerMotor.setPower(0.35);
                    currentState = State.intakeState;
                    intakePath(poseEstimate, 0 ,0);
                    //TODO: end limelight and get limelightOffsets save them to a variable see below
//                    if (limeLightTimer.milliseconds() > 0) {
//                        currentState = State.intakeState;
//                        intakePath(poseEstimate, 0 ,0);
//
//                    }
                    break;
                case intakeState:
                    if (!drive.isBusy()) {
//                        intake.activeIntake.flipToTransfer();
                        moveLift(slidePos);
                        if(!colorSensor.hasSample() || colorSensor.opposingColor()){
                            currentState = State.spitState;
                            intake.extendToTransfer();
                            intake.activeIntake.rollerMotor.setPower(0.35);
                            spitPath(poseEstimate);
                        }else{
                            //TODO: uncomment this when BSTEM
                            if(runTime.milliseconds() > 28000){
                                intake.activeIntake.motorRollerOff();
                                intake.activeIntake.flipToTransfer();
                                currentState = State.IDLE;
                            }else{
                                intake.activeIntake.flipToTransfer();
                                currentState = State.scoreState;
                                scorePath(poseEstimate);
                            }
                            //TODO: commet out when BSTEM
//                            currentState = State.scoreState;
//                            scorePath(poseEstimate);
                        }
                    }
                    break;
                case parkState:
                    if (!drive.isBusy()) {
                        arm.toScoreSpecimen();
                        moveLift(0);
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();

            poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("LL offset", 0);
            telemetry.update();
        }
    }

    enum State {
        initialSamplesState,
        pickupState,
        limelightState,
        intakeState,
        spitState,
        scoreState,
        parkState,
        IDLE            // Our bot will enter the IDLE state when done
    }

    public void pickUpPath(Pose2d robotPose){
        TrajectorySequence trajSeq;
        if(sample == 5) {
            trajSeq = drive.trajectorySequenceBuilder(robotPose)
                    .waitSeconds(0.15)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        moveLift(2000);
                        arm.readyForTransfer();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        moveLift(slidePos);
                        arm.readyForTransfer();
                    })
                    .splineToLinearHeading(new Pose2d(-22.5, cord1,Math.toRadians(0)), Math.toRadians(0))
                    .build();
        }else{
            trajSeq = drive.trajectorySequenceBuilder(robotPose)
                    .waitSeconds(0.15)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        moveLift(2000);
                        arm.readyForTransfer();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        moveLift(slidePos);
                        arm.readyForTransfer();
                    })
                    .splineTo(new Vector2d(-22.5, cord2), Math.toRadians(0))
                    .build();
        }
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void intakePath(Pose2d robotPose, double xOffset, double yOffset){
        TrajectorySequence trajSeq;
        if(sample == 5){
            trajSeq = drive.trajectorySequenceBuilder(robotPose)
                    .lineToConstantHeading(new Vector2d(-18, cord1))// plus is for vision offsets
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        intake.activeIntake.flipDownFull();
                    })
                    //.waitSeconds(0.75)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
//                        intake.extendoFullExtend();
                        intake.activeIntake.motorRollerOnToIntake();

                        moveExtendo(0.07);
                    })
                    .waitSeconds(0.4)
                    .turn(Math.toRadians(4.5))
                    .turn(Math.toRadians(-4.5))
                    .build();
        }else if(sample == 6){
            trajSeq = drive.trajectorySequenceBuilder(robotPose)
                    .lineToConstantHeading(new Vector2d(-18, cord2))// plus is for vision offsets
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                        moveExtendo(0.07);
                        intake.extendoFullExtend();
                        intake.activeIntake.flipDownFull();
                    })
                    //.waitSeconds(0.75)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        intake.extendoFullExtend();
//                        moveExtendo(0.07);
                        intake.activeIntake.motorRollerOnToIntake();
                    })
                    .waitSeconds(0.4)
                    .turn(Math.toRadians(4.5))
                    .turn(Math.toRadians(-4.5))
                    .build();
        }else{
            trajSeq = drive.trajectorySequenceBuilder(robotPose)
                    .lineToConstantHeading(new Vector2d(-18, cord3))// plus is for vision offsets
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        moveExtendo(0.07);
                        intake.activeIntake.flipDownFull();
                    })
                        //.waitSeconds(0.75)
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                            intake.extendoFullExtend();
                            intake.activeIntake.motorRollerOnToIntake();
                        })
                        .waitSeconds(0.4)
                        .turn(Math.toRadians(4.5))
                        .turn(Math.toRadians(-4.5))
                        .build();
        }
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void scorePath(Pose2d robotPose){
        tryCount = 0;
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .addTemporalMarker(0, () -> {
                    intake.activeIntake.flipToTransfer();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(0.5, () -> {
                    intake.extendToTransfer();
                })
                //transfer
                .addTemporalMarker(0.9, () -> {
                    intake.activeIntake.rollerMotor.setPower(1);
                })

                .addTemporalMarker(1.1, () -> {
                    arm.closeClaw();

                })
                .addTemporalMarker(1.5, () -> {
                    //arm.wrist.setPosition(OuttakeConstants.ArmPositions.GRABBING_SPEC.getWristPos());
                    moveLift(1900);
                    intake.activeIntake.motorRollerOff();
                    arm.toScoreSample();
                })
                .lineToLinearHeading(new Pose2d(-40, robotPose.getY(),Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(scorePosX+1, scorePosY+1, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(scorePosX, scorePosY, Math.toRadians(45)))
                .waitSeconds(0.1)
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void spitPath(Pose2d robotPose){
        tryCount ++;
        TrajectorySequence trajSeq;
        if(sample == 5) {
            trajSeq = drive.trajectorySequenceBuilder(robotPose)
                    .lineToConstantHeading(new Vector2d(-22.5, cord1))
                    .build();
        }else{
            trajSeq = drive.trajectorySequenceBuilder(robotPose)
                    .lineToConstantHeading(new Vector2d(-22.5, cord2))
                    .build();
        }
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public void parkPath(Pose2d robotPose){
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(robotPose)
                .waitSeconds(0.1)
                .addTemporalMarker(0.7, () -> {
                    moveLift(0);
                })
                .splineToLinearHeading(new Pose2d(-20,-8,Math.toRadians(180)), Math.toRadians(0))
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
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