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

    public enum SampleTypes{
        NONE(new double[]{57,95,114}, "NONE"),
        YELLOW(new double[]{215,287,110}, "YELLOW"),
        BLUE(new double[]{55,99,156}, "BLUE"),
        RED(new double[]{173,126,85}, "RED");

        public final double[] color;
        public final String name;

        SampleTypes(double[] color, String name){
            this.color = color;
            this.name = name;
        }
    }
}
