package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * Finds the NEAREST 3" yellow ball with a Limelight 3A color pipeline and reports:
 *   - distance from the camera lens to the ball
 *   - heading error (how many degrees the robot must turn to face the ball)
 *
 * WHY NOT JUST USE result.getTx() / getTy()?
 * Because the camera is pitched down. When a camera is tilted, the simple
 * "distance = height / tan(mountAngle + ty)" formula is only exact for targets
 * dead-center horizontally, and tx is only the true heading when pitch == 0.
 * This file builds the actual 3D ray through the pixel, rotates it by the mount
 * pitch, and intersects it with the plane containing the ball's center. That is
 * correct everywhere in the image, which is what makes it "reliable."
 *
 * COORDINATE CONVENTIONS
 *   Limelight tx: + means target is RIGHT of the crosshair
 *   Limelight ty: + means target is ABOVE the crosshair
 *   Robot frame:  F = forward, R = right, U = up
 *   CAMERA_PITCH_DEG is NEGATIVE when the camera is tilted downward.
 *   Returned heading error is + when the robot must turn RIGHT (clockwise).
 *
 * IMPORTANT: this math assumes the pipeline's crosshair is at the image center
 * (the default). If you "calibrate" the crosshair off-center in the web UI, tx/ty
 * become relative to that point and these numbers will be biased.
 */
@TeleOp(name = "Limelight Yellow Ball Tracker", group = "Vision")
public class ChaseBall extends LinearOpMode {

    // ==================================================================
    // 1. MEASURE THESE ON YOUR ROBOT. Everything else depends on them.
    // ==================================================================

    /** Pipeline index you configured as the yellow-ball color pipeline. */
    private static final int PIPELINE_INDEX = 1;

    /** Height of the CENTER OF THE LENS above the floor, in inches. */
    private static final double CAMERA_HEIGHT_IN = 10.5;

    /** Downward tilt. Negative = pointing down. Fine-tune this with the procedure in the notes. */
    private static final double CAMERA_PITCH_DEG = -19.0;

    /** Lens position relative to the robot's center of rotation, in inches. */
    private static final double CAMERA_FORWARD_IN = 9.5;   // + = ahead of center
    private static final double CAMERA_RIGHT_IN   = 0.0;   // + = right of center

    /** Game element. A ball resting on the floor has its center one radius up. */
    private static final double BALL_DIAMETER_IN = 2.5;
    private static final double BALL_RADIUS_IN   = BALL_DIAMETER_IN / 2.0;

    // ==================================================================
    // 2. LIMELIGHT 3A OPTICS (from the spec sheet; don't change unless
    //    you switch resolution or run ChArUco calibration)
    // ==================================================================
    private static final double HFOV_DEG = 54.5;
    private static final double VFOV_DEG = 42.0;
    private static final int    IMG_W_PX = 640;
    private static final int    IMG_H_PX = 480;

    // ==================================================================
    // 3. SANITY GATES - reject junk detections before they reach the drivetrain
    // ==================================================================
    private static final double MIN_VALID_DIST_IN = 5.0;
    private static final double MAX_VALID_DIST_IN = 120.0;
    /** Max allowed disagreement between geometry distance and apparent-size distance (0.40 = 40%). */
    private static final double SIZE_CHECK_TOLERANCE = 0.40;
    /** Reject stale frames (ms). */
    private static final long MAX_STALENESS_MS = 150;
    /** How many good frames in a row before we call the target "locked". */
    private static final int LOCK_FRAMES = 3;

    // Smoothing (0 = no smoothing, 0.7 = heavy). Exponential moving average.
    private static final double SMOOTHING = 0.5;

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addLine("Ready. Camera height " + CAMERA_HEIGHT_IN
                + "in, pitch " + CAMERA_PITCH_DEG + "deg");
        telemetry.update();

        waitForStart();

        int goodFrames = 0;
        double smoothDist = 0, smoothHeading = 0;

        while (opModeIsActive()) {
            BallTarget nearest = null;
            String rejectReason = "no result";

            LLResult result = limelight.getLatestResult();

            if (result == null) {
                rejectReason = "null result (is the pipeline a COLOR pipeline?)";
            } else if (!result.isValid()) {
                rejectReason = "no targets";
            } else if (result.getStaleness() > MAX_STALENESS_MS) {
                rejectReason = "stale data (" + result.getStaleness() + " ms)";
            } else {
                List<LLResultTypes.ColorResult> blobs = result.getColorResults();
                if (blobs == null || blobs.isEmpty()) {
                    rejectReason = "no color blobs";
                } else {
                    // Solve every blob, keep the closest one that passes all gates.
                    for (LLResultTypes.ColorResult blob : blobs) {
                        BallTarget t = solve(
                                blob.getTargetXDegrees(),
                                blob.getTargetYDegrees(),
                                blob.getTargetArea());
                        if (t == null) continue;
                        if (t.cameraDistanceIn < MIN_VALID_DIST_IN
                                || t.cameraDistanceIn > MAX_VALID_DIST_IN) continue;
                        if (t.sizeDisagreement > SIZE_CHECK_TOLERANCE) continue;
                        if (nearest == null || t.cameraDistanceIn < nearest.cameraDistanceIn) {
                            nearest = t;
                        }
                    }
                    if (nearest == null) rejectReason = "all blobs failed sanity gates";
                }
            }

            if (nearest != null) {
                goodFrames++;
                if (goodFrames == 1) {                 // first hit: seed the filter
                    smoothDist = nearest.cameraDistanceIn;
                    smoothHeading = nearest.headingErrorDeg;
                } else {
                    smoothDist = SMOOTHING * smoothDist + (1 - SMOOTHING) * nearest.cameraDistanceIn;
                    smoothHeading = SMOOTHING * smoothHeading + (1 - SMOOTHING) * nearest.headingErrorDeg;
                }
            } else {
                goodFrames = 0;
            }

            boolean locked = goodFrames >= LOCK_FRAMES;

            telemetry.addData("Status", locked ? "TARGET LOCKED" : "SEARCHING (" + rejectReason + ")");
            if (locked) {
                telemetry.addData("Distance (camera -> ball)", "%.1f in", smoothDist);
                telemetry.addData("Heading error", "%.1f deg  (%s)",
                        smoothHeading, smoothHeading > 0 ? "turn RIGHT" : "turn LEFT");
                telemetry.addLine();
                telemetry.addData("  ground dist from robot center", "%.1f in", nearest.robotGroundDistanceIn);
                telemetry.addData("  forward / right of camera", "%.1f / %.1f in",
                        nearest.forwardIn, nearest.rightIn);
                telemetry.addData("  raw tx / ty", "%.2f / %.2f deg", nearest.txDeg, nearest.tyDeg);
                telemetry.addData("  size cross-check error", "%.0f%%", nearest.sizeDisagreement * 100);
            }
            telemetry.update();

            // Example use:
            // if (locked) turnPower = clip(smoothHeading * 0.02, -0.4, 0.4);
        }
    }

    // ==================================================================
    // THE MATH
    // ==================================================================

    /** Everything we know about one candidate ball. Distances in inches, angles in degrees. */
    public static class BallTarget {
        public double txDeg, tyDeg;
        public double forwardIn, rightIn;          // ball center relative to the LENS, on the floor plane
        public double cameraDistanceIn;            // straight-line lens -> ball center  (the headline number)
        public double cameraGroundDistanceIn;      // lens -> ball, measured along the floor
        public double robotGroundDistanceIn;       // robot center -> ball, along the floor
        public double headingErrorDeg;             // + = turn right, measured from robot center
        public double sizeDisagreement;            // 0.0 = apparent size perfectly matches geometry
    }

    /**
     * Turn one (tx, ty, area) detection into a real-world position.
     * Returns null if the detection is geometrically impossible (ray at or above the horizon).
     */
    private BallTarget solve(double txDeg, double tyDeg, double areaPercent) {
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);

        // Step 1: undo the projection. tx/ty are angles, so tan() recovers the
        // normalized image coordinates of the blob center on a unit focal plane.
        double xn = Math.tan(Math.toRadians(txDeg));   // + right
        double yn = Math.tan(Math.toRadians(tyDeg));   // + up

        // Step 2: build the ray in camera axes, then rotate by the mount pitch
        // into robot axes (F = forward, R = right, U = up). No roll assumed.
        double rayF = Math.cos(pitch) - yn * Math.sin(pitch);
        double rayR = xn;
        double rayU = Math.sin(pitch) + yn * Math.cos(pitch);

        // Step 3: the ray must be heading downward to ever hit the ball's center plane.
        if (rayU > -1e-4) return null;

        // Step 4: intersect with the horizontal plane at the ball's center height.
        double dropIn = CAMERA_HEIGHT_IN - BALL_RADIUS_IN;
        if (dropIn <= 0) return null;                 // camera mounted below ball center
        double scale = dropIn / (-rayU);

        double forward = scale * rayF;
        double right   = scale * rayR;

        BallTarget t = new BallTarget();
        t.txDeg = txDeg;
        t.tyDeg = tyDeg;
        t.forwardIn = forward;
        t.rightIn = right;
        t.cameraGroundDistanceIn = Math.hypot(forward, right);
        t.cameraDistanceIn = Math.sqrt(forward * forward + right * right + dropIn * dropIn);

        // Step 5: shift the origin from the lens to the robot's center of rotation,
        // so the heading error is what the DRIVETRAIN actually needs to turn.
        double robotForward = forward + CAMERA_FORWARD_IN;
        double robotRight   = right   + CAMERA_RIGHT_IN;
        t.robotGroundDistanceIn = Math.hypot(robotForward, robotRight);
        t.headingErrorDeg = Math.toDegrees(Math.atan2(robotRight, robotForward));

        // Step 6: independent cross-check using apparent size. A 3" sphere has a
        // known angular diameter at a known range, so if the two methods disagree
        // badly the blob is probably not a ball (or it is partially occluded).
        t.sizeDisagreement = sizeDisagreement(areaPercent, t.cameraDistanceIn);
        return t;
    }

    /** |sizeDistance - geometryDistance| / geometryDistance, or 0 if area is unusable. */
    private double sizeDisagreement(double areaPercent, double geometryDistIn) {
        if (areaPercent <= 0.0001 || geometryDistIn <= 0) return 0;

        // Bounding box of a circle is square, so width = sqrt(area in pixels).
        double areaPx = (areaPercent / 100.0) * IMG_W_PX * IMG_H_PX;
        double widthPx = Math.sqrt(areaPx);

        // Focal length in pixels, derived from the published horizontal FOV.
        double focalPx = (IMG_W_PX / 2.0) / Math.tan(Math.toRadians(HFOV_DEG / 2.0));

        double sizeDistIn = (BALL_DIAMETER_IN * focalPx) / widthPx;
        return Math.abs(sizeDistIn - geometryDistIn) / geometryDistIn;
    }

    // Unused but kept for reference: vertical focal length, if you ever want it.
    @SuppressWarnings("unused")
    private static double focalYPx() {
        return (IMG_H_PX / 2.0) / Math.tan(Math.toRadians(VFOV_DEG / 2.0));
    }
}
