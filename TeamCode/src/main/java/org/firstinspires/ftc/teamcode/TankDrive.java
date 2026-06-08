package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TankDrive", group = "TeleOp")
public class TankDrive extends OpMode {

    private DcMotor leftDrive, rightDrive;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor intake;
    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        double leftJoyY = -gamepad1.left_stick_y;
        double rightJoyY = -gamepad1.right_stick_y;

        if (gamepad1.left_bumper) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }
        if (gamepad1.right_bumper) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }
//        leftDrive.setPower(leftJoyY);
//        rightDrive.setPower(rightJoyY);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("leftJoyY: ",leftJoyY);
        telemetry.addData("rightJoyY: ",rightJoyY);
        telemetry.update();

    }

}
