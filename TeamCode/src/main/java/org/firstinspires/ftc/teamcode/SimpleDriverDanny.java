package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class SimpleDriverDanny {
    public static class Poses {
        //region shared poses
        public static Pose TOP_RIGHT_CORNER = new Pose(134, 134, Math.toRadians(0));
        public static Pose BOTTOM_LEFT_CORNER = new Pose (10,10, Math.toRadians(180));
    }
    //region Enums
    public enum Alliance {
        RED,
        BLUE
    }

    public enum DriveMode {
        FIELD,
        ROBOT
    }
    //endregion

    //region Static Variables
    public static Pose lastKnownPose;
    public static Alliance currentAlliance;
    public static DriveMode currentDriveMode;
    public static ElapsedTime idleTimer;

//    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
//    private static final Style historyLook = new Style("", "#4CAF50", 0.75);
//    private static final Style limelightLook = new Style("", "#FF9800", 0.75);
    //endregion

    //region Class Members
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
//    private Limelight3A limelight;
    private Telemetry telemetry;
    private Follower follower; // part of the Pedro Pathing package, follows the path

    public boolean debug = false;
    private boolean slowMode = false;
    private PIDFController headingPIDFController;


//    private double relocalizePedroX;
//    private double relocalizePedroY;
    //endregion

    //region Constructors
    public SimpleDriverDanny(HardwareMap hardwareMap, Telemetry telemetryFromOpMode,
                             Alliance alliance, Pose startingPose) {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0); // april tag pipeline
//        limelight.setPollRateHz(100);
//        limelight.start();

        // this is our constructor that gets called like this from our autos:  driver = new DriverDanny(hardwareMap, telemetry, DriverDanny.Poses.RED_FAR_START_POSE);
        // think of this like our "init" but for the DriverDanny specifically
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

//        PanelsField.INSTANCE.getField().setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        telemetry = telemetryFromOpMode;
        currentAlliance = alliance;
        currentDriveMode = DriveMode.FIELD;

        idleTimer = new ElapsedTime();

        // initialize a new PIDF controller using the heading coefficients we already tuned for auto
        headingPIDFController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //endregion

    //region Update Functions
    public void update() { // THIS MUST ALWAYS GO IN YOUR OPMODE LOOP EVERY CALL
        follower.update(); // this will just update the Pedro Pathing following but can add additional steps if we need to later
//        this.updateLimeLight(); // should update our limelight every loop

        // if the robot has changed position, reset the timer so we can track how long we've been idle
        if (lastKnownPose != null &&
                (
                    lastKnownPose.distanceFrom(this.getPose()) > 0.5
                    || Math.abs(Math.toDegrees(lastKnownPose.getHeading()) - Math.toDegrees(this.getPose().getHeading())) > 0.5
                )
            ) {
            idleTimer.reset();
        }

        lastKnownPose = this.getPose();


        telemetry.addData("CurrentXPos", lastKnownPose.getX());
        telemetry.addData("CurrentYPos", lastKnownPose.getY());
        telemetry.addData("CurrentHeading", Math.toDegrees(lastKnownPose.getHeading()));
        telemetry.addData("CurrentAlliance", currentAlliance.toString());
        telemetry.addData("CurrentDriveMode", currentDriveMode.toString());
        telemetry.addData("SlowModeEnabled", slowMode);
        telemetry.addData("IdleTimer", idleTimer.milliseconds());


//        if (debug) { drawDebugField(); }
    }

//    public void updateLimeLight() {
//        double currentHeading = this.follower.getHeading();
////        limelight.updateRobotOrientation(Math.toDegrees(currentHeading)+90); // subtract 90 degrees here for pedropathing heading conversion
//
//        if (shouldRelocalize
//                && lastRelocalizeTimer.milliseconds() > 5000
//                && idleTimer.milliseconds() > 100) {
//
//            // Learned that Tx, Ty, and Ta are degrees of error from tag, not meters.
//            LLResult result = limelight.getLatestResult();
//            if (result != null && result.isValid()) {
//                Pose3D botpose = result.getBotpose_MT2();
//
//                if (botpose != null) {
//                    // convert from meters to inches and adjust for 0,0 origin like pedropathing instead of -72,-72 that limelight uses
//                    relocalizePedroX = (botpose.getPosition().y * 39.3700787) + 72.0; // x and y are intentionally flipped here
//                    relocalizePedroY = -(botpose.getPosition().x * 39.3700787) + 72.0;
//
//                    Pose newPedroPose = new Pose(relocalizePedroX, relocalizePedroY, currentHeading);
//
//                    if (Math.abs(botpose.getPosition().z) < 0.1
//                            && relocalizePedroX > 0 && relocalizePedroX < 144
//                            && relocalizePedroY > 0 && relocalizePedroY < 144) {
//                        this.follower.setPose(newPedroPose);
//                        lastRelocalizeTimer.reset();
//                    }
//                }
//            }
//        }
//    }

    //endregion

    //region Drive Functions
    public void drive(double joyY, double joyX, double rotate) {
        if (currentDriveMode == DriveMode.FIELD) {
            fieldCentricDrive(joyY, joyX, rotate);
        } else {
            robotCentricDrive(joyY, joyX, rotate);
        }
    }
    private void robotCentricDrive(double forward, double strafe, double rotate) {
        // followed brogan's tutorial on this
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        // this is useful when precision driving is needed (like parking adjustments)
        if (slowMode) { maxSpeed = 0.5; }

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void fieldCentricDrive(double joyY, double joyX, double rotate) {
        double fieldX;
        double fieldY;
        double currentRobotHeading = this.follower.getHeading();

        // change joystick "driver intent" to field X and Y intent
        if (currentAlliance == Alliance.BLUE) {
            fieldX = -joyY; // if pressing down on joyY on blue alliance, intent is to increase field X
            fieldY = joyX; // if pressing right on joyX on blue alliance, intent is to increase field Y
        } else {
            fieldX = joyY; // if pressing up on joyY on red alliance, intent is to increase field X
            fieldY = -joyX; // if pressing left on joyX on red alliance, intent is to increase field Y
        }

        // now translate field intent to robot actual direction
        double robotX = fieldX * Math.cos(currentRobotHeading)
                + fieldY * Math.sin(currentRobotHeading);

        double robotY = fieldX * Math.sin(currentRobotHeading)
                - fieldY * Math.cos(currentRobotHeading);

        // If our currentDriveMode is robotCentric, ignore the translations and use original paramters
        if (currentDriveMode == DriveMode.ROBOT) {
            this.robotCentricDrive(joyY, joyX, rotate);
        } else {
            this.robotCentricDrive(robotX, robotY, rotate);
        }
    }
    //endregion

    //region Auto-aim Functions
//    public void updateHeadingErrorForAutoAimTrig() {
//            Pose currentPose = this.getPose();
//            Pose goalPose;
//
//        if (currentAlliance == Alliance.RED) {
//            goalPose = Poses.RED_GOAL_AIMING_POSE;
//        } else {
//            goalPose = Poses.BLUE_GOAL_AIMING_POSE;
//        }
//
//        // get the "vector" from current robot position to goal position
//        double dx = goalPose.getX() - currentPose.getX();
//        double dy = goalPose.getY() - currentPose.getY();
//
//        // use atan2 to get heading from x-axis to goal in radians
//        double targetHeading = Math.atan2(dy, dx);
//
//        double turnDirection = MathFunctions.getTurnDirection(currentPose.getHeading(), targetHeading);
//        double angleDifference = MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), targetHeading);
//        double headingError = turnDirection * angleDifference;
//
//        lastAutoAimHeadingError = headingError;
//    }


    //endregion

    //region Other Helper Functions
//    public double getCurrentDistanceFromGoal() {
//        if (currentAlliance == Alliance.RED) {
//            return this.getPose().distanceFrom(Poses.RED_GOAL_POSE);
//        } else {
//            return this.getPose().distanceFrom(Poses.BLUE_GOAL_POSE);
//        }
//    }

    public void swapCurrentAlliance() {
        if (currentAlliance == Alliance.BLUE) {
            currentAlliance = Alliance.RED;
        } else {
            currentAlliance = Alliance.BLUE;
        }
    }

    public void swapCurrentDriveMode() {
        if (currentDriveMode == DriveMode.FIELD) {
            currentDriveMode = DriveMode.ROBOT;
        } else {
            currentDriveMode = DriveMode.FIELD;
        }
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public Pose getPose() {
        return follower.getPose(); // this will just return the current pose from Pedro Pathing but can add additional steps if we need to later
    }

    public boolean isBusy() {
        return follower.isBusy(); // checks to see if PedroPathing is still busy and returns that result
    }

    public void moveToPose(Pose newPose, boolean holdEnd) {
        PathChain newPath = follower.pathBuilder()
                .addPath(new BezierLine(getPose(), newPose))
                .setLinearHeadingInterpolation(getPose().getHeading(), newPose.getHeading(), 0.8)
                .setGlobalDeceleration()
                .build();

        follower.followPath(newPath,0.89, holdEnd); // start the robot moving towards the new pose immediately
    }


    public void abortPath() {
        if (this.isBusy()) {
            follower.breakFollowing();
        }
    }

    // to localize right now, make sure to drive into your alliance corner and make sure the robot is facing upfield towards the goals (90 degrees)


//    public void resetHeadingTo90Degrees() {
//        Pose newHeadingPose = new Pose(this.getPose().getX(), this.getPose().getY(), Math.toRadians(90));
//        this.follower.setPose(newHeadingPose);
//    }

//    private void drawDebugField() {
//        Pose pose = follower.getPose();
//        FieldManager panelsField = PanelsField.INSTANCE.getField();
//        PoseHistory poseHistory = follower.getPoseHistory();
//
//        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
//            return;
//        }
//
//        // Draw pose history
//        panelsField.setStyle(historyLook);
//        if (poseHistory != null) {
//            int size = poseHistory.getXPositionsArray().length;
//            for (int i = 0; i < size - 1; i++) {
//                panelsField.moveCursor(poseHistory.getXPositionsArray()[i], poseHistory.getYPositionsArray()[i]);
//                panelsField.line(poseHistory.getXPositionsArray()[i + 1], poseHistory.getYPositionsArray()[i + 1]);
//            }
//        }
//
//        // Draw robot
//        panelsField.setStyle(robotLook);
//        panelsField.moveCursor(pose.getX(), pose.getY());
//        panelsField.circle(9); // ROBOT_RADIUS
//
//        Vector v = pose.getHeadingAsUnitVector();
//        v.setMagnitude(v.getMagnitude() * 9);
//        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
//        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
//
//        panelsField.setStyle(robotLook);
//        panelsField.moveCursor(x1, y1);
//        panelsField.line(x2, y2);
//
//        // Draw limelight relocalization position
//        if (relocalizePedroX != 0.0 && relocalizePedroY != 0.0) {
//            panelsField.setStyle(limelightLook);
//            panelsField.moveCursor(relocalizePedroX, relocalizePedroY);
//            panelsField.circle(5); // Draw a slightly smaller circle for the limelight pose
//        }
//
//        panelsField.update();
//    }
    //endregion
}