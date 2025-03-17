package org.firstinspires.ftc.teamcode.mechanisms.outtake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;

public class Outtake {
    // SLIDES
    private PIDController controller;
    public DcMotorEx outtakeSlideRight;
    public DcMotorEx outtakeSlideLeft;
    private static double p, i, d; //has to be tuned
    private static double f; // usually mass moved * constant G

    // OTHER
    Telemetry telemetry;
    GamepadMapping controls;
    public TouchSensor touchSensor;

    public Outtake(HardwareMap hardwareMap, int direction, double inP, double inI, double inD, double inF, Telemetry telemetry,
    GamepadMapping controls){
        outtakeSlideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        outtakeSlideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        outtakeSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        if(direction == 0){
            outtakeSlideLeft.setDirection(DcMotorEx.Direction.FORWARD);
            outtakeSlideRight.setDirection(DcMotorEx.Direction.REVERSE);
        }else{
            outtakeSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
            outtakeSlideRight.setDirection(DcMotorEx.Direction.FORWARD);
        }

        controller = new PIDController(p,i,d);

        p = inP; i = inI; d = inD; f = inF;

        this.telemetry = telemetry;
        this.controls = controls;
    }

    // this is for J-Unit testing only
    public Outtake(DcMotorEx slidesMotorLeft, DcMotorEx slidesMotorRight, PIDController controller) {
        this.outtakeSlideLeft = slidesMotorLeft;
        this.outtakeSlideRight = slidesMotorRight;
        this.controller = controller;
    }

    public void moveTicks(double target) {
        controller.setPID(p,i,d);
        int pos = outtakeSlideLeft.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double power = pid + f;
        outtakeSlideRight.setPower(power);
        outtakeSlideLeft.setPower(power);
    }

    public void changePIDF(double inP, double inI, double inD, double inF){
        p = inP; i = inI; d = inD; f = inF;
    }

    public int getPos(){
        return outtakeSlideLeft.getCurrentPosition();
    }

    public void extendToLowBasket() {
        moveTicks(OuttakeConstants.SlidePositions.LOW_BASKET.getSlidePos());
    }

    public void extendToHighBasket() {
        moveTicks(OuttakeConstants.SlidePositions.HIGH_BASKET.getSlidePos());
    }

    public void extendToSpecimenHighRackLow() {
        moveTicks(OuttakeConstants.SlidePositions.SPECIMEN_HIGH_RACK_LOW.getSlidePos()); // tune target obviously
    }

    public void extendToSpecimenHighRackHigh() {
        moveTicks(OuttakeConstants.SlidePositions.SPECIMEN_HIGH_RACK_HIGH.getSlidePos()); // tune target obviously
    }

    public void upToTransfer() {
        moveTicks(OuttakeConstants.SlidePositions.TRANSFER.getSlidePos());
    }

    public void hang() {
        moveTicks(OuttakeConstants.SlidePositions.HANGING_HIGH.getSlidePos());
    }

    public void returnToRetracted() {
        moveTicks(OuttakeConstants.SlidePositions.RETRACTED.getSlidePos());
    }

    public void resetHardware() {
        returnToRetracted();
    }

    public void resetEncoders() {
        // reset slide motor encoders
        outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotorsToTeleOpMode() {
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
