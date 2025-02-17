package org.firstinspires.ftc.teamcode.mechanisms.intake.archived;

public class ClawConstants {
    // open, closed
    private static final double[] clawPositions = {.2, .72};

    // 90 degrees (sample "up and down" -> original pos & init transfer), 180 degrees (sample "left and right" -> transfer)
    private static final double[] wristPositions = {.6, 1}; // wrist also changed by joystick

    // transfer, hover, picking up, outtaking (vertical)
    private static final double[] v4bPositions = { .2, .9, 1, .43};

    public enum ClawStates {
        BASE_STATE(clawPositions[0], wristPositions[0], v4bPositions[0], "BASE STATE"),
        TRANSFER(clawPositions[0], wristPositions[1], v4bPositions[0], "TRANSFER"),
        HOVER_OVER_SAMPLES(clawPositions[0], wristPositions[0], v4bPositions[1], "HOVERING"),
        OUTTAKING(clawPositions[0], wristPositions[0], v4bPositions[3], "OUTTAKING"),
        PICKING_UP_SAMPLE(clawPositions[1], wristPositions[0], v4bPositions[2], "PICKING SAMPLE");

        private final double clawPos;
        private final double wristPos;
        private final double v4bPos;
        private final String debug;

        ClawStates(double clawPos, double wristPos, double v4bPos, String debug) {
            this.clawPos = clawPos;
            this.wristPos = wristPos;
            this.v4bPos = v4bPos;
            this.debug = debug;
        }

        public double getClawPos() { return clawPos; }
        public double getWristPos() { return wristPos; }
        public double getV4bPos() { return v4bPos; }
        public String toString() { return debug;}
    }
}
