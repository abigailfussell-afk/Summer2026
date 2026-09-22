package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeamBreak {

    private DigitalChannel BeamBreak;

    public void init(HardwareMap hardwareMap) {
        BeamBreak = hardwareMap.get(DigitalChannel.class, "beambreak");
        BeamBreak.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getBeamBreakState() {
        return BeamBreak.getState();
    }

}
