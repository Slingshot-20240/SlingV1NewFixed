package org.firstinspires.ftc.teamcode.mechanisms.outtake;

public class OuttakeConstants {

    // Retracted, low basket, high basket, below spec high rack, score spec low, score spec high, hang high, hang low, transfer
    private static double[] slidePositions = {0, 700, 2000, 0, 700, 2200, 1600, 300}; // mini extend
    public enum SlidePositions {
        RETRACTED(slidePositions[0]),
        LOW_BASKET(slidePositions[1]), // probably could work for hang
        HIGH_BASKET(slidePositions[2]),
        SPECIMEN_HIGH_RACK_LOW(slidePositions[3]),
        SPECIMEN_HIGH_RACK_HIGH(slidePositions[4]),
        BASE_STATE(slidePositions[0]),
        TRANSFER(slidePositions[7]),
        HANGING_HIGH(slidePositions[5]),
        HANGING_LOW(slidePositions[6]);

        private final double slidePos;

        SlidePositions(double slidePos) {
            this.slidePos = slidePos;
        }
        public double getSlidePos() { return slidePos; }
    }

    // Transfer, going up, deposit, grab spec, safe deposit
    private static double[] armPositions = {.2, .13, .9, .32, .9};
    // Transfer, deposit, pick spec
    private static double[] wristPositions = {.95, 0, 0.95};
    // open, closed
    private static double[] clawPositions = {0, .55};

    public enum ArmPositions {
        RETRACTED(armPositions[0], wristPositions[0], clawPositions[0]),
        BASKET(armPositions[2], wristPositions[1], clawPositions[1]),
        SPECIMEN_HIGH_RACK(armPositions[2], wristPositions[1], clawPositions[1]),
        TRANSFERING(armPositions[1], wristPositions[0], clawPositions[0]),
        GRABBING_SPEC(armPositions[3], wristPositions[2], clawPositions[0]),
        SAFE_SAMP(armPositions[4], wristPositions[1], clawPositions[1]);

        private final double armPos;
        private final double wristPos;
        private final double clawPos;

        ArmPositions(double armPos, double wristPos, double clawPos) {
            this.armPos = armPos;
            this.wristPos = wristPos;
            this.clawPos = clawPos;
        }

        public double getArmPos() {
            return armPos;
        }

        public double getWristPos() {
            return wristPos;
        }

        public double getClawPos() {
            return clawPos;
        }
    }

}
