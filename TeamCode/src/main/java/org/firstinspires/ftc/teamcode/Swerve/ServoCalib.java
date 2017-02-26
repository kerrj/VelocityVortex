package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Swerve.Core.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Swerve.Core.Constants;
import org.firstinspires.ftc.teamcode.Swerve.Core.FTCSwerve;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 10/21/2016.
 */
@TeleOp
public class ServoCalib extends Robot {
    AbsoluteEncoder lfe;
    AbsoluteEncoder rfe;
    AbsoluteEncoder lbe;
    AbsoluteEncoder rbe;
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
        lastLoop=System.nanoTime()/1.0E6;
        lfe=new AbsoluteEncoder(0,lfa);
        lbe=new AbsoluteEncoder(0,lba);
        rfe=new AbsoluteEncoder(0,rfa);
        rbe=new AbsoluteEncoder(0,rba);

    }

    @Override
    public void loop() {
        telemetry.addData("lf",Math.toDegrees(lfe.getAngle()));
        telemetry.addData("rf",Math.toDegrees(rfe.getAngle()));
        telemetry.addData("lb",Math.toDegrees(lbe.getAngle()));
        telemetry.addData("rb",Math.toDegrees(rbe.getAngle()));
    }
}
