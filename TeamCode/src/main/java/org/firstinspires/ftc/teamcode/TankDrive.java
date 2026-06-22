package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TankDrive", group = "TeleOp")
public class TankDrive extends OpMode {

    private DcMotor leftDrive, rightDrive;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor intake;
    private CRServo leftServo;

    private CRServo rightServo;
    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftServo.setDirection(DcMotor.Direction.FORWARD);
        rightServo.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftServo.setMode(CRServo.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        double leftJoyY = -gamepad1.left_stick_y;
        double rightJoyY = -gamepad1.right_stick_y;

        leftDrive.setPower(leftJoyY);
        rightDrive.setPower(rightJoyY);

        if (gamepad1.left_bumper) {
            intake.setPower(1);
            leftServo.setPower(1);
            rightServo.setPower(1);
        }
        else {
            intake.setPower(0);
            leftServo.setPower(0);
            rightServo.setPower(0);
        }
        if (gamepad1.right_bumper) {
            intake.setPower(-1);
            leftServo.setPower(-1);
            rightServo.setPower(-1);
        }
        else {
            intake.setPower(0);
            leftServo.setPower(0);
            rightServo.setPower(0);
        }
//        leftDrive.setPower(leftJoyY);
//        rightDrive.setPower(rightJoyY);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("leftJoyY: ",leftJoyY);
        telemetry.addData("rightJoyY: ",rightJoyY);
        telemetry.addData("intake", intake.getPower());
        telemetry.addData("leftServo", leftServo.getPower());
        telemetry.addData("rightServo", rightServo.getPower());
        telemetry.update();

    }

}
