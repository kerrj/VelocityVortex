package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CameraStuff.EyeOfSauron;

/**
 * Created by Justin on 11/4/2016.
 */
@TeleOp
@Disabled
public class EyeOfSauronTest extends OpMode {
    Servo neck, base;
    EyeOfSauron cameraTower;

    public void init(){
        neck=hardwareMap.servo.get("neck");
        base=hardwareMap.servo.get("base");
        cameraTower=new EyeOfSauron(base, neck);
        cameraTower.lookAt(0,0);
    }
    public void loop(){
        cameraTower.lookAt(gamepad1.left_stick_y*90,gamepad1.left_stick_x*90);
    }
    public void stop(){
        
    }
}
