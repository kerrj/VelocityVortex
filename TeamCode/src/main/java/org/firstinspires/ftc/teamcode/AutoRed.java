package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * Created by Justin on 9/10/2016.
 */
public class AutoRed extends LinearOpMode {
    private static final int WHEEL_DIAMETER=4;
    private static final double WHEEL_CIRCUMFERENCE=WHEEL_DIAMETER*Math.PI;
    private static final int COUNTS_PER_REVOLUTION=1444;
    private static final int BUTTON_COLOR_SENSOR_ADDRESS=0x6c;//MAKE SURE THIS MATCHES THE ADDRESS IN THE MODERN ROBOTICS INSPECTOR
    private static final int LINE_COLOR_SENSOR_ADDRESS=0x42;//MAKE SURE THIS MATCHES THE ADDRESS IN THE MODERN ROBOTICS INSPECTOR
    private static final int STARTING_DISTANCE_TO_WALL=50;

    private DcMotor left;
    private DcMotor right;
    private GyroSensor gyro;
    private UltrasonicSensor rangeSensor;
    private ColorSensor buttonColor;
    private ColorSensor lineColor;
    private FTCVuforia vuforia;
    private Servo wheelServo;
    private long beaconDetectTime;
    private int encoderCountsleft;
    private int encoderCountsRight;

    @Override
    public void runOpMode() throws InterruptedException {
        left=hardwareMap.dcMotor.get("left");
        right=hardwareMap.dcMotor.get("right");
        wheelServo=hardwareMap.servo.get("wheel");
        buttonColor=hardwareMap.colorSensor.get("buttonColor");
        lineColor=hardwareMap.colorSensor.get("lineColor");
        gyro=hardwareMap.gyroSensor.get("gyro");
        rangeSensor=hardwareMap.ultrasonicSensor.get("ultrasonic");


        right.setDirection(DcMotor.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        buttonColor.setI2cAddress(I2cAddr.create8bit(BUTTON_COLOR_SENSOR_ADDRESS));
        lineColor.setI2cAddress(I2cAddr.create8bit(LINE_COLOR_SENSOR_ADDRESS));

        //initialize vuforia
//        vuforia=new FTCVuforia(FtcRobotControllerActivity.getActivity());
//        vuforia.addTrackables("FTC_2016-17.xml");

        gyro.calibrate();
        waitForStart();
        while(gyro.isCalibrating()) {
        }

        //first we need to drive to the wall and turn to align parallel
        encoderCountsleft=left.getCurrentPosition();
        encoderCountsRight=right.getCurrentPosition();
        while(left.getCurrentPosition()<inchesToCounts(STARTING_DISTANCE_TO_WALL)+encoderCountsleft){
            left.setPower(.5);
            right.setPower(.5);
        }
        left.setPower(0);
        right.setPower(0);

        //turn left to align with wall
        sleep(300);



    }



    /**
    Method to convert inches to encoder counts
     @param inches The number of inches to convert
    @return integer number of encoder counts
     **/
    public int inchesToCounts(double inches){
        return (int)(inches/WHEEL_CIRCUMFERENCE)*COUNTS_PER_REVOLUTION;
    }
}
