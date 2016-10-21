package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Swerve.Core.FTCSwerve;

/**
 * Created by Justin on 10/15/2016.
 */
public class Robot extends OpMode {

    public DcMotor lfm,lbm,rfm,rbm;
    public Servo lf,lb,rf,rb;
    public AnalogInput lfa,lba,rfa,rba;
    public FTCSwerve swerveDrive;



    @Override
    public void init() {
        lfm=hardwareMap.dcMotor.get("lfm");
        rfm=hardwareMap.dcMotor.get("rfm");
        lbm=hardwareMap.dcMotor.get("lbm");
        rbm=hardwareMap.dcMotor.get("rbm");
        lf=hardwareMap.servo.get("lf");
        lb=hardwareMap.servo.get("lb");
        rf=hardwareMap.servo.get("rf");
        rb=hardwareMap.servo.get("rb");
        lfa=hardwareMap.analogInput.get("lfa");
        lba=hardwareMap.analogInput.get("lba");
        rfa=hardwareMap.analogInput.get("rfa");
        rba=hardwareMap.analogInput.get("rba");
        swerveDrive=new FTCSwerve(lfa,rfa,lba,rba,lfm,rfm,lbm,rbm,lf,rf,lb,rb,16,16);
    }

    @Override
    public void loop() {

    }
}
