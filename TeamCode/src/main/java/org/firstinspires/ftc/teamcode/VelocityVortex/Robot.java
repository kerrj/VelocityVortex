package org.firstinspires.ftc.teamcode.VelocityVortex;

import android.view.GestureDetector;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
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
    public static final double NECK_FLAT=.45;
    public static final double CAP_RIGHT_IN=.4;
    public static final double CAP_LEFT_IN=.08;
    public static final double CAP_RIGHT_OUT=.5;
    public static final double CAP_LEFT_OUT=.65;
    public static final double CAP_RIGHT_HOLD=.8;
    public static final double CAP_LEFT_HOLD=.35;
    public static final int SLIDE_DOWN=0;
    public static final int SLIDE_UP=20200;
    public static final double SHOOTER_DOWN=.6;
    public static final double SHOOTER_UP=0;
    public static final double SWEEPER_INTAKE=1;
    public static final double SWEEPER_OUTAKE=-1;
    public static final double SWEEPER_STOP=0;

    public DcMotor lfm,lbm,rfm,rbm,slideMotor,shootLeft,shootRight,sweeper;
    public Servo lf,lb,rf,rb;
    public Servo buttonWheel, capLeft, capRight,shootServo,neck;
    public AnalogInput lfa,lba,rfa,rba;
    public FTCSwerve swerveDrive;
    public AbsoluteEncoder lfe,rfe,rbe,lbe;
//    public DataLogger dataLogger;
    public FTCVuforia vuforia;
//    public ModernRoboticsI2cRangeSensor leftRangeMeter,rightRangeMeter;

    public int slideStartPosition;//if start position is not 0

    public double lastLoop;
    public GyroSensor gyro;


    @Override
    public void init() {
//        dataLogger=DataLogger.create(50);
        lfm=hardwareMap.dcMotor.get("lfm");
        rfm=hardwareMap.dcMotor.get("rfm");
        lbm=hardwareMap.dcMotor.get("lbm");
        rbm=hardwareMap.dcMotor.get("rbm");
        gyro=hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        sweeper=hardwareMap.dcMotor.get("sweeper");
        shootLeft=hardwareMap.dcMotor.get("shootLeft");
        shootRight=hardwareMap.dcMotor.get("shootRight");
        shootRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shootLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf=hardwareMap.servo.get("lf");
        lb=hardwareMap.servo.get("lb");
        rf=hardwareMap.servo.get("rf");
        rb=hardwareMap.servo.get("rb");
        shootServo=hardwareMap.servo.get("shootServo");
        shootServo.setPosition(SHOOTER_DOWN);
        slideMotor=hardwareMap.dcMotor.get("slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideStartPosition=slideMotor.getCurrentPosition();
        lf.setPosition(0.5);
        lb.setPosition(0.5);
        rf.setPosition(0.5);
        rb.setPosition(0.5);
        neck=hardwareMap.servo.get("neck");
        buttonWheel=hardwareMap.servo.get("buttonWheel");
        capLeft=hardwareMap.servo.get("capLeft");
        capRight=hardwareMap.servo.get("capRight");
        buttonWheel.setPosition(WHEEL_IN);
        neck.setPosition(NECK_FLAT);
        capRight.setPosition(CAP_RIGHT_IN);
        capLeft.setPosition(CAP_LEFT_IN);
        lfa=hardwareMap.analogInput.get("lfa");
        lba=hardwareMap.analogInput.get("lba");
        rfa=hardwareMap.analogInput.get("rfa");
        rba=hardwareMap.analogInput.get("rba");
        lfe=new AbsoluteEncoder(Constants.FL_OFFSET, lfa);
        rfe=new AbsoluteEncoder(Constants.FR_OFFSET, rfa);
        rbe=new AbsoluteEncoder(Constants.BR_OFFSET, rba);
        lbe=new AbsoluteEncoder(Constants.BL_OFFSET, lba);
        swerveDrive=new FTCSwerve(lfa,rfa,lba,rba,lfm,rfm,lbm,rbm,lf,rf,lb,rb,14,14);
//        dataLogger.mapHardware(this);
//        I2cDeviceSynch i=hardwareMap.i2cDeviceSynch.get("leftRanger");
//        leftRangeMeter=new ModernRoboticsI2cRangeSensor(i);
//        I2cDeviceSynch i2=hardwareMap.i2cDeviceSynch.get("rightRanger");
//        rightRangeMeter=new ModernRoboticsI2cRangeSensor(i2);
//        I2cAddr left=I2cAddr.create8bit(0x10);
//        leftRangeMeter.setI2cAddress(left);
//        I2cAddr right=I2cAddr.create8bit(0x28);
//        rightRangeMeter.setI2cAddress(right);
        lastLoop=System.nanoTime();
    }

    @Override
    public void loop(){
        swerveDrive.refreshValues();
        telemetry.addData("LoopTime",System.nanoTime()/1E6-lastLoop);
        lastLoop=System.nanoTime()/1E6;

    }
    @Override
    public  void stop(){
        lf.setPosition(0.5);
        lb.setPosition(0.5);
        rf.setPosition(0.5);
        rb.setPosition(0.5);
        capRight.setPosition(CAP_RIGHT_HOLD);
        capLeft.setPosition(CAP_LEFT_HOLD);
    }
}
