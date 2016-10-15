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
public class Servoo extends OpMode{ServoModule module;
    DcMotor motor;
    AnalogInput a;
    AbsoluteEncoder encoder;
    Servo steerservo;

    @Override
    public void init() {
        motor=hardwareMap.dcMotor.get("motor");
        steerservo=hardwareMap.servo.get("servo");
        a=hardwareMap.analogInput.get("ai");
        encoder=new AbsoluteEncoder(Constants.FR_OFFSET,a);
        module=new ServoModule(steerservo, encoder);
    }

    @Override
    public void loop() {
        Vector direction=new Vector(gamepad1.left_stick_y,gamepad1.left_stick_x);
        if(direction.getMagnitude()>.1) {
            module.set(direction.getAngle());
        }else{
            module.set(direction.getAngle());
        }
        telemetry.addData("Angle",Math.toDegrees(encoder.getAngle()));
        module.update();
    }
}
