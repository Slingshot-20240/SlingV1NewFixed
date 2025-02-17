package org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class GoofyLocalizer {
    // Encoder motors
    private DcMotorEx parallelEncoder;
    private DcMotorEx perpendicularEncoder;

    // IMU for heading
    private BNO055IMU imu;

    // Constants
    private final double TICKS_PER_INCH = 8192.0; // Replace with your actual value
    private final double PARALLEL_X_OFFSET = 0.0; // Replace with your parallel encoder's X offset
    private final double PARALLEL_Y_OFFSET = 5.0; // Replace with your parallel encoder's Y offset
    private final double PERPENDICULAR_X_OFFSET = 0.0; // Replace with your perpendicular encoder's X offset
    private final double PERPENDICULAR_Y_OFFSET = -5.0; // Replace with your perpendicular encoder's Y offset

    // Robot's current pose (position and heading)
    private Pose2d pose = new Pose2d(0, 0, 0);

    // Encoder state tracking
    private double lastParallelPosition = 0.0; // Previous parallel encoder value
    private double lastPerpendicularPosition = 0.0; // Previous perpendicular encoder value

    public GoofyLocalizer(HardwareMap hardwareMap) {
        // Initialize encoders
        parallelEncoder = hardwareMap.get(DcMotorEx.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotorEx.class, "perpendicularEncoder");

        // Reset encoders
        parallelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set encoders to run without encoders (to allow manual position tracking)
        parallelEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        perpendicularEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);
    }

    // Update position
    public void update() {
        // Get the current heading
        double headingRadians = getRobotHeading();

        // Get the current encoder positions
        double parallelPosition = parallelEncoder.getCurrentPosition() / TICKS_PER_INCH;
        double perpendicularPosition = perpendicularEncoder.getCurrentPosition() / TICKS_PER_INCH;

        // Calculate deltas (change in encoder values since last update)
        double deltaParallel = parallelPosition - lastParallelPosition;
        double deltaPerpendicular = perpendicularPosition - lastPerpendicularPosition;

        // Update last positions
        lastParallelPosition = parallelPosition;
        lastPerpendicularPosition = perpendicularPosition;

        // Calculate robot-relative deltas
        double robotDeltaX = deltaParallel;
        double robotDeltaY = deltaPerpendicular;

        // Adjust for encoder offsets
        double adjustedDeltaX = robotDeltaX - (PERPENDICULAR_Y_OFFSET * deltaPerpendicular - PARALLEL_Y_OFFSET * deltaParallel);
        double adjustedDeltaY = robotDeltaY + (PERPENDICULAR_X_OFFSET * deltaPerpendicular - PARALLEL_X_OFFSET * deltaParallel);

        // Convert robot-relative deltas to global frame
        double headingCos = Math.cos(headingRadians);
        double headingSin = Math.sin(headingRadians);

        double globalDeltaX = adjustedDeltaX * headingCos - adjustedDeltaY * headingSin;
        double globalDeltaY = adjustedDeltaX * headingSin + adjustedDeltaY * headingCos;

        // Update the robot's pose
        double newX = pose.getX() + globalDeltaX;
        double newY = pose.getY() + globalDeltaY;

        pose = new Pose2d(newX, newY, headingRadians);
    }

    // Get the robot's heading in radians
    private double getRobotHeading() {
        Orientation angles = imu.getAngularOrientation();
        return AngleUnit.normalizeRadians(angles.firstAngle);
    }

    // Set the known global position and heading
    public void setPose(Pose2d newPose) {
        pose = newPose;

        // Reset encoder deltas to prevent jumps in position
        lastParallelPosition = parallelEncoder.getCurrentPosition() / TICKS_PER_INCH;
        lastPerpendicularPosition = perpendicularEncoder.getCurrentPosition() / TICKS_PER_INCH;
    }

    // Get the current pose
    public Pose2d getPose() {
        return pose;
    }
}