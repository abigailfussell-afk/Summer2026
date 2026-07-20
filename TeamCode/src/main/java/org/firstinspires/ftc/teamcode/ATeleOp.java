package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ATeleOp", group = "TeleOp")
public class  ATeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private SimpleDriverDanny driver;
    private DcMotor intake;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SimpleDriverDanny.Alliance startingAlliance = SimpleDriverDanny.Alliance.BLUE;
        Pose startingPose = SimpleDriverDanny.Poses.TOP_RIGHT_CORNER;

        if (SimpleDriverDanny.currentAlliance != null) {
            startingAlliance = SimpleDriverDanny.currentAlliance;
        }

        if (SimpleDriverDanny.lastKnownPose != null) {
            startingPose = SimpleDriverDanny.lastKnownPose;
        }

        driver = new SimpleDriverDanny(hardwareMap,
                telemetry,
                startingAlliance,
                startingPose);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    @Override
    public void loop() {
        driver.update();

        double joyY = -gamepad1.left_stick_y; // leaving this inverted so it works for robotCentricDrive (and we adjust for it on fieldCentricDrive)
        double joyX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        //region Gamepad1
        if (gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {
            driver.swapCurrentAlliance(); // lets us swap our alliance (for auto-aim / driver testing)
        }

        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            driver.swapCurrentDriveMode();
        }

        if (gamepad1.bWasPressed()) {
            driver.toggleSlowMode(); // allows us to cut robot movement speed in half when precision is needed
        }

//        if (gamepad1.backWasPressed()) {
//            driver.resetHeadingTo90Degrees(); // ONLY USE THIS IF THE HEADING IS ABSOLUTELY BROKEN!!
//        }

//        if (gamepad1.yWasPressed()) {
//            shooter.activateKickstand();
//        }
        //endregion

        //region Gamepad2
        if (gamepad2.left_bumper || gamepad1.left_bumper) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }

        //endregion

        // this drive function uses field-centric driving by default unless toggled to robot
        driver.drive(joyY, joyX, rotate);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("HeadingLock", headingLock);
        telemetry.addData("intake",intake.getPower());
        telemetry.update();
    }
}