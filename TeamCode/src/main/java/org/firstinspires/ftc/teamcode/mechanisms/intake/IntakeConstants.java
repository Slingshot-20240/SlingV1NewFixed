package org.firstinspires.ftc.teamcode.mechanisms.intake;

public class IntakeConstants {

    // pivoted up, pivoted down, transfer pos, clearing samples, spec mode
    // axon programmed for 0-255, 66 PMW (inverted)
    private static final double[] pivotPositions = {.27, .89, .22, .5, .35};

    // right linkage in, right linkage extended, outtaking, spec mode
    // axon programmed for 0-255, 75 PMW
    private static final double[] rightLinkagePositions = {.33, .035, .23, .2};

    // left linkage in, left linkage extended
    // axon programmed for 0-255, 75 PMW
    private static final double[] leftLinkagePositions = {.33, .035, .23, .2};

    private static final double[] v4bPositions = {0,0}; //TODO: you need to fix this there was nothing there and it brok
    public enum ActiveIntakeStates {
        FULLY_RETRACTED(pivotPositions[0], rightLinkagePositions[0], leftLinkagePositions[0]), // pivoted up, idle back roller, retracted
        CLEARING(pivotPositions[3], rightLinkagePositions[1], leftLinkagePositions[1]),
        FULLY_EXTENDED(pivotPositions[1], rightLinkagePositions[1], leftLinkagePositions[1]), // pivoted down, idle back roller, extended
        TRANSFER(pivotPositions[2], rightLinkagePositions[2], leftLinkagePositions[2]), // pivoted up, back roller push, retracted
        SPEC_MODE(pivotPositions[4], rightLinkagePositions[3], leftLinkagePositions[3]);

        private final double pivotPos;
        private final double rLinkagePos;
        private final double lLinkagePos;

        ActiveIntakeStates(double pivotPos, double rLinkagePos, double lLinkagePos) {
            this.pivotPos = pivotPos;
            this.rLinkagePos = rLinkagePos;
            this.lLinkagePos = lLinkagePos;
        }

        public double pivotPos() { return pivotPos; }
        public double rLinkagePos() { return rLinkagePos; }
        public double lLinkagePos() { return lLinkagePos; }

    }
    //TODO: there is no such thing as v4b positions

    public enum v4bActiveStates {
        FULLY_RETRACTED(v4bPositions[0], rightLinkagePositions[0], leftLinkagePositions[0]),
        CLEARING(v4bPositions[1], rightLinkagePositions[1], leftLinkagePositions[1]),
        FULLY_EXTENDED(v4bPositions[1], rightLinkagePositions[1], leftLinkagePositions[1]), // pivoted down, idle back roller, extended
        TRANSFER(v4bPositions[1], rightLinkagePositions[0], leftLinkagePositions[0]), // pivoted up, back roller push, retracted
        OUTTAKING(v4bPositions[0], rightLinkagePositions[2], leftLinkagePositions[2]),
        HOVERING(v4bPositions[1], rightLinkagePositions[1], leftLinkagePositions[1]);
        private double v4bPos;
        private final double rLinkagePos;
        private final double lLinkagePos;
        v4bActiveStates (double v4bPos, double rightLinkagePos, double leftLinkagePos) {
            this.v4bPos = v4bPos;
            this.rLinkagePos = rightLinkagePos;
            this.lLinkagePos = leftLinkagePos;
        }

        public double v4bPos() { return v4bPos; }
        public double rLinkagePos() { return rLinkagePos; }
        public double lLinkagePos() { return lLinkagePos; }
    }
}
