package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class BeamBreakFr extends OpMode {

    BeamBreak beam = new BeamBreak();

    public void init() {
        beam.init(hardwareMap);

        }

        public void loop () {
            telemetry.addData("BeamBreak", beam.getBeamBreakState());

        }
    }

