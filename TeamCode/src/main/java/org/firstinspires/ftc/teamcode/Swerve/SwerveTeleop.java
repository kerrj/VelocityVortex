package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Justin on 10/14/2016.
 */
@TeleOp
public class SwerveTeleop extends OpMode {

    DcMotor left,right;
    Servo lf,lb,rf,rb;
    AnalogInput lfa,lba,rfa,rba;
    FTCSwerve swerveDrive;
    @Override
    public void init() {
        left=hardwareMap.dcMotor.get("left");
        right=hardwareMap.dcMotor.get("right");
        lf=hardwareMap.servo.get("lf");
        lb=hardwareMap.servo.get("lb");
        rf=hardwareMap.servo.get("rf");
        rb=hardwareMap.servo.get("rb");
        lfa=hardwareMap.analogInput.get("lfa");
        lba=hardwareMap.analogInput.get("lba");
        rfa=hardwareMap.analogInput.get("rfa");
        rba=hardwareMap.analogInput.get("rba");
        swerveDrive=new FTCSwerve(lfa,rfa,lba,rba,left,right,lf,rf,lb,rb);
    }

    @Override
    public void loop() {
        Vector direction=new Vector(gamepad1.left_stick_y,gamepad1.left_stick_x);
        if(direction.getMagnitude()>.1){
            swerveDrive.driveTowards(direction,direction.getMagnitude());
        }else{
            swerveDrive.driveTowards(direction,0);
        }

        if(Math.abs(gamepad1.right_stick_x)>.1){
            swerveDrive.rotate(gamepad1.right_stick_x);
        }
        swerveDrive.update(false);
    }
}
