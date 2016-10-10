package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Justin on 10/10/2016.
 */
@TeleOp
public class Servoo extends OpMode{SwerveModule module;
    DcMotor motor;
    AnalogInput a;
    AbsoluteEncoder encoder;
    Servo steerservo;

    @Override
    public void init() {
        motor=hardwareMap.dcMotor.get("motor");
        steerservo=hardwareMap.servo.get("servo");
        a=hardwareMap.analogInput.get("ai");
        encoder=new AbsoluteEncoder(0,a);
        module=new SwerveModule(motor,steerservo,encoder,16,16);
    }

    @Override
    public void loop() {
        double angle=gamepad1.left_stick_y*Math.PI;
        telemetry.addData("angle",Double.toString(angle));
        module.set(Math.PI,1);
        module.update(false);
    }
}
