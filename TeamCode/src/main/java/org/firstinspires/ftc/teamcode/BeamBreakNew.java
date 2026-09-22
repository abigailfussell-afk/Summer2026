package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Beam Break Test", group = "Sensor")
public class BeamBreakNew extends LinearOpMode {
    private DigitalChannel beamBreak;

    @Override
    public void runOpMode() {

        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");

        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // getState() returns TRUE if the beam is unbroken (light is reaching the receiver)
            // getState() returns FALSE if the beam is broken (an object is blocking the light)
            boolean isUnbroken = beamBreak.getState();

            if (!isUnbroken) {
                telemetry.addData("Object Detected", "YES (Beam Broken)");
            } else {
                telemetry.addData("Object Detected", "NO (Beam Unbroken)");
            }

            telemetry.addData("Raw State", isUnbroken);
            telemetry.update();
        }
    }
}