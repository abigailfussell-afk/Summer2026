package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "SkyNet", group = "TeleOp")
public class SkyNet extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // april tag pipeline
        limelight.setPollRateHz(100);
        limelight.start();

    }

    @Override
    public void loop(){
        LLResult result = limelight.getLatestResult();

        LLResultTypes.ColorResult closest = null;

        if(result != null && result.isValid()) {
            for(LLResultTypes.ColorResult target: result.getColorResults()){

                if(closest == null ||
                    target.getTargetArea()> closest.getTargetArea()) {
                    closest = target;
                }
            }
        }

        if(closest != null) {
            telemetry.addData("tx", closest.getTargetXDegrees());
            telemetry.addData("ty", closest.getTargetYDegrees());
            telemetry.addData("area", closest.getTargetArea());

        }


    }
}
