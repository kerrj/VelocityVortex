package org.firstinspires.ftc.teamcode.Swerve;
import org.firstinspires.ftc.teamcode.Swerve.Constants;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by hunai on 9/30/2016.
 */
@TeleOp
public class EncoderTest extends OpMode {
//    SwerveDrive swerve;
    static AbsoluteEncoder enc1;
    static AnalogInput analoginput1;
    public void init(){
        analoginput1 =hardwareMap.analogInput.get("ai");
        enc1 = new AbsoluteEncoder(0, analoginput1);
    }
    public void loop(){
        telemetry.addData("endcoder angle", enc1.getAngle());
        telemetry.addData("endcoder voltage", enc1.getVoltage());
        telemetry.addData("endcoder max voltage", enc1.getMaxVoltage());



    }
    public void stop(){

    }
}
