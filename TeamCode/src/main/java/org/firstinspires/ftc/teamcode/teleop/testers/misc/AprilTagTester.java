package org.firstinspires.ftc.teamcode.teleop.testers.misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.aprilTag.AprilTags;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous
@Disabled
public class AprilTagTester extends LinearOpMode {
    OpenCvCamera camera;
    AprilTags aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    public Servo leftFinger = null;
    public Servo rightFinger = null;
    public DcMotor slide = null;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;
    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTags(tagsize, fx, fy, cx, cy);
        leftFinger = hardwareMap.get(Servo.class, "leftFinger");
        rightFinger = hardwareMap.get(Servo.class, "rightFinger");
        leftFinger.scaleRange(0.0, 1.0);
        rightFinger.scaleRange(0.0, 1.0);
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        /*TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .addTemporalMarker(0, () -> {
//                    leftFinger.setPosition(0.4);
//                    rightFinger.setPosition(0.6);*/
//                })
//                .addTemporalMarker(1, () -> {
//                    slide.setTargetPosition(2500);
//                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slide.setPower(-1);
//                })
//                .forward(49)
//                .strafeRight(8)
//                .addTemporalMarker(3.75, () -> {
//                    leftFinger.setPosition(0.9);
//                    rightFinger.setPosition(0.1);
//                    slide.setTargetPosition(1000);
//                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slide.setPower(1);
//                })
//                /*.strafeLeft(8)
//                .forward(14)
//                .strafeRight(40)
//                .addTemporalMarker(8, () -> {
//                    leftFinger.setPosition(0.4);
//                    rightFinger.setPosition(0.6);*/
//                })
//                .addTemporalMarker(10, () -> {
//                    slide.setTargetPosition(1700);
//                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slide.setPower(-1);
//                })
//                .addTemporalMarker(13, () -> {
//                    slide.setTargetPosition(0);
//                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slide.setPower(1);
//                })

                //.build();
//        Trajectory left = drive.trajectoryBuilder(myTrajectory.end())
//                .strafeLeft(60)
//                .build();
//        Trajectory middle = drive.trajectoryBuilder(myTrajectory.end())
//                .strafeLeft(30)
//                .build();
//        Trajectory right = drive.trajectoryBuilder(myTrajectory.end())
//                .strafeLeft(10)
//                .build();
        while (!isStarted() && !isStopRequested()){
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0) {
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else {
                telemetry.addLine("Don't see tag of interest :(");
                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        String parking = "";

        if(tagOfInterest.id == LEFT) { //ONE
            parking = "left";
        } else if (tagOfInterest.id == MIDDLE) { //TWO
            parking = "middle";
        } else if (tagOfInterest.id == RIGHT) { //THREE
            parking = "right";

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        if(isStopRequested()) return;

//        drive.followTrajectorySequence(myTrajectory);
//        if (parking == "left") {
//            drive.followTrajectory(left);
//        }
//        else if (parking == "right") {
//            drive.followTrajectory(right);
//        }
//        else if (parking == "middle") {
//            drive.followTrajectory(middle);
//        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

