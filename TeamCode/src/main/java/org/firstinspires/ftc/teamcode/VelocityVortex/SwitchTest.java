package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by Justin on 12/2/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class SwitchTest extends OpMode {
    AnalogInput s;
    @Override
    public void init() {
        s=hardwareMap.analogInput.get("switch");
    }

    @Override
    public void loop() {
        telemetry.addData("voltage",s.getVoltage());
    }
}
