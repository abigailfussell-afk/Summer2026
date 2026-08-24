package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Finds the nearest yellow ball and reports distance + heading error.
 *
 * HOW IT WORKS (two ideas, that's it):
 *
 *   1. HEADING ERROR is just tx. Limelight already tells us how many degrees
 *      left or right the ball is. Positive tx = ball is to the right.
 *
 *   2. DISTANCE comes from ty. A ball that is far away appears high in the
 *      picture; a ball that is close appears low. Instead of doing trigonometry,
 *      we MEASURE this once with a tape measure and write the numbers down.
 *      The code just looks up the answer between the numbers we measured.
 *
 * The Limelight picks the nearest ball for us: set the pipeline's Sort Mode to
 * "Largest" in the web interface, because the closest ball looks the biggest.
 */
@TeleOp(name = "Distance From Pollen", group = "Vision")
public class DistanceFromPollen extends LinearOpMode {

    private Limelight3A limelight;

    /**
     * THE CALIBRATION TABLE - this is the homework.
     *
     * Put the ball at a known distance, read ty off the telemetry, write it here.
     * Every row is {ty, distance in inches}. Go from FAR (high ty) to CLOSE (low ty).
     * Six or seven rows is plenty. These numbers are made up - replace them!
     */
    private static final double[][] TY_TO_INCHES = {
            { -2.0,  72.0 },
            { -5.5,  60.0 },
            { -9.0,  48.0 },
            { -13.5, 36.0 },
            { -18.0, 24.0 },
            { -21.0, 18.0 },
            { -24.5, 12.0 },
    };

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                double distance = lookUpDistance(ty);

                telemetry.addData("Distance", "%.1f inches", distance);
                telemetry.addData("Heading error", "%.1f degrees", tx);
                telemetry.addData("(raw ty, for calibrating)", "%.2f", ty);
            } else {
                telemetry.addLine("No ball found");
            }
            telemetry.update();
        }
    }

    /**
     * Finds the distance for a given ty by looking it up in our measured table.
     * If ty falls between two rows, we split the difference proportionally.
     * This is called "linear interpolation."
     */
    private double lookUpDistance(double ty) {
        // Ball is farther than anything we measured
        if (ty >= TY_TO_INCHES[0][0]) {
            return TY_TO_INCHES[0][1];
        }
        // Ball is closer than anything we measured
        int last = TY_TO_INCHES.length - 1;
        if (ty <= TY_TO_INCHES[last][0]) {
            return TY_TO_INCHES[last][1];
        }

        // Walk the table until we find the two rows that ty sits between
        for (int i = 0; i < last; i++) {
            double tyHigh = TY_TO_INCHES[i][0];
            double tyLow = TY_TO_INCHES[i + 1][0];

            if (ty <= tyHigh && ty >= tyLow) {
                double distHigh = TY_TO_INCHES[i][1];
                double distLow = TY_TO_INCHES[i + 1][1];

                // How far are we between the two rows? 0.0 = at the top row, 1.0 = at the bottom row
                double fraction = (tyHigh - ty) / (tyHigh - tyLow);

                return distHigh + fraction * (distLow - distHigh);
            }
        }
        return 0; // should never happen
    }
}
