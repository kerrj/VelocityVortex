package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CameraStuff.EyeOfSauron;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCVuforia;
import org.firstinspires.ftc.teamcode.Logging.DataLogger;
import org.firstinspires.ftc.teamcode.Swerve.Core.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Swerve.Core.Constants;
import org.firstinspires.ftc.teamcode.Swerve.Core.FTCSwerve;

/**
 * Created by Justin on 10/15/2016.
 */
public class Robot extends OpMode {
    public static final double WHEEL_IN=.95;
    public static final double WHEEL_OUT=.3;
    public static final double NECK_FLAT=.4;
    public DcMotor lfm,lbm,rfm,rbm;
    public Servo lf,lb,rf,rb;
    public AnalogInput lfa,lba,rfa,rba;
    public FTCSwerve swerveDrive;
    public Servo neck,buttonWheel;
    public AbsoluteEncoder lfe,rfe,rbe,lbe;
//    public DataLogger dataLogger;
    public FTCVuforia vuforia;
//    public ModernRoboticsI2cRangeSensor leftRangeMeter,rightRangeMeter;



    @Override
    public void init() {
//        dataLogger=DataLogger.create(50);
        lfm=hardwareMap.dcMotor.get("lfm");
        rfm=hardwareMap.dcMotor.get("rfm");
        lbm=hardwareMap.dcMotor.get("lbm");
        rbm=hardwareMap.dcMotor.get("rbm");
        lf=hardwareMap.servo.get("lf");
        lb=hardwareMap.servo.get("lb");
        rf=hardwareMap.servo.get("rf");
        rb=hardwareMap.servo.get("rb");
        lf.setPosition(.5);
        lb.setPosition(.5);
        rf.setPosition(.5);
        rb.setPosition(.5);
        neck=hardwareMap.servo.get("neck");
        buttonWheel=hardwareMap.servo.get("buttonWheel");
        buttonWheel.setPosition(WHEEL_IN);
        neck.setPosition(NECK_FLAT);
        lfa=hardwareMap.analogInput.get("lfa");
        lba=hardwareMap.analogInput.get("lba");
        rfa=hardwareMap.analogInput.get("rfa");
        rba=hardwareMap.analogInput.get("rba");
        lfe=new AbsoluteEncoder(Constants.FL_OFFSET, lfa);
        rfe=new AbsoluteEncoder(Constants.FR_OFFSET, rfa);
        rbe=new AbsoluteEncoder(Constants.BR_OFFSET, rba);
        lbe=new AbsoluteEncoder(Constants.BL_OFFSET, lba);
        swerveDrive=new FTCSwerve(lfa,rfa,lba,rba,lfm,rfm,lbm,rbm,lf,rf,lb,rb,16,16);
//        dataLogger.mapHardware(this);
//        I2cDeviceSynch i=hardwareMap.i2cDeviceSynch.get("leftRanger");
//        leftRangeMeter=new ModernRoboticsI2cRangeSensor(i);
//        I2cDeviceSynch i2=hardwareMap.i2cDeviceSynch.get("rightRanger");
//        rightRangeMeter=new ModernRoboticsI2cRangeSensor(i2);
//        I2cAddr left=I2cAddr.create8bit(0x10);
//        leftRangeMeter.setI2cAddress(left);
//        I2cAddr right=I2cAddr.create8bit(0x28);
//        rightRangeMeter.setI2cAddress(right);
    }

    @Override
    public void loop(){

    }
    @Override
    public  void stop(){
        lf.setPosition(.5);
        lb.setPosition(.5);
        rf.setPosition(.5);
        rb.setPosition(.5);
    }
}
