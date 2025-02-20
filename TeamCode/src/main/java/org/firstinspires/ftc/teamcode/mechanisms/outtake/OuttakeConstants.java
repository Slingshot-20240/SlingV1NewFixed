package org.firstinspires.ftc.teamcode.mechanisms.outtake;

public class OuttakeConstants {
    // deposit: 0
    // tilt: .5477610423136522
    // transfer ready: .7231126346979301

    // transfer ready, tilt, deposit
    //.95 for transfer previously
    private static double[] leftBucketPositions = {.982, .8, .58};
    private static double[] rightBucketPositions = {1, .773, .603};

    // Retracted, low basket, high basket, below spec high rack, score spec low, score spec high
    private static double[] slidePositions = {0, 700, 2000, 0, 700, 700, 500}; // mini extend
    public enum SlidePositions {
        RETRACTED(slidePositions[0]),
        LOW_BASKET(slidePositions[1]), // probably could work for hang
        HIGH_BASKET(slidePositions[2]),
        SPECIMEN_HIGH_RACK_LOW(slidePositions[3]),
        SPECIMEN_HIGH_RACK_HIGH(slidePositions[4]),
        HUMAN_PLAYER(slidePositions[0]),
        BASE_STATE(slidePositions[0]),
        //GRABBING_SPEC(slidePositions[5]),
        HANGING_HIGH(slidePositions[4]),
        HANGING_LOW(slidePositions[5]);

        private final double slidePos;

        SlidePositions(double slidePos) {
            this.slidePos = slidePos;
        }
        public double getSlidePos() { return slidePos; }
    }

    public enum BucketPositions {
        TRANSFER_READY(leftBucketPositions[0], rightBucketPositions[0]),
        TILT(leftBucketPositions[1], rightBucketPositions[1]),
        DEPOSIT(leftBucketPositions[2], rightBucketPositions[2]);

        private final double leftBucketPos;
        private final double rightBucketPos;

        BucketPositions(double lBucketPos, double rBucketPos) {
            this.leftBucketPos = lBucketPos;
            this.rightBucketPos = rBucketPos;
        }

        public double getRightBucketPos() {
            return rightBucketPos;
        }
        public double getLeftBucketPos() {
            return leftBucketPos;
        }
    }

    // Transfer, going up, deposit, grab spec
    private static double[] armPositions = {.42, .32, 1, .52}; // TODO: tune these values
    // Transfer, deposit
    private static double[] wristPositions = {.94, 0}; // TODO: tune these values
    // open, closed
    private static double[] clawPositions = {0, .24}; // TODO: tune these values

    public enum ArmPositions {
        RETRACTED(armPositions[0], wristPositions[0], clawPositions[0]),
        BASKET(armPositions[2], wristPositions[1], clawPositions[1]),
        SPECIMEN_HIGH_RACK(armPositions[2], wristPositions[1], clawPositions[1]),
        TRANSFERING(armPositions[1], wristPositions[0], clawPositions[0]),
        GRABBING_SPEC(armPositions[3], wristPositions[0], clawPositions[0]);

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
