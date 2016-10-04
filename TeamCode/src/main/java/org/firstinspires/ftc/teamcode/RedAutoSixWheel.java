package org.firstinspires.ftc.teamcode;

import android.text.style.WrapTogetherSpan;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by hunai on 9/23/2016.
 */
@Autonomous
public class RedAutoSixWheel extends OpMode {
    final double WHEEL_IN=0;
    final double WHEEL_OUT=.7;
    final double DISTANCE_FROM_WALL=5;
    ColorSensor floorSensor;
    ColorSensor beaconSensor;
    Servo buttonServo;
    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor rangeSensor;
    DcMotor right;
    DcMotor left;
    State robotState;
    boolean nearButton;
    Autonomous_Methods methods=new Autonomous_Methods();
    int startPosition;

    /*the following variables exist to set the starting position of the encoders and button position ONCE
    while looping, as opposed to setting them every iteration of the loop
     */
    boolean resetPosition=true;
    boolean resetButton=true;

    int beaconsPressed=0;//used to stop the robot from indefinitely pressing buttons

    //define the discrete robot state possibilities here
    enum State{
        DriveStraight,Turn1,DriveToWall, TurnToBeacon, DriveToFirstBeacon,PressBeacon,DriveToBeacon,Stop
    }

    @Override
    public void init() {
        right=hardwareMap.dcMotor.get("right");
        left=hardwareMap.dcMotor.get("left");
        floorSensor=hardwareMap.colorSensor.get("floorSensor");
        beaconSensor=hardwareMap.colorSensor.get("beaconSensor");
        beaconSensor.enableLed(false);
        buttonServo=hardwareMap.servo.get("buttonServo");
        gyro=hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        I2cDeviceSynch i=hardwareMap.i2cDeviceSynch.get("rangeSensor");
        rangeSensor=new ModernRoboticsI2cRangeSensor(i);
        I2cAddr f=I2cAddr.create8bit(0x42);
        I2cAddr b=I2cAddr.create8bit(0x6c); // must use create8bit to work
        beaconSensor.setI2cAddress(b);
        floorSensor.setI2cAddress(f);
        left.setDirection(DcMotor.Direction.REVERSE);
        floorSensor.enableLed(true);
        buttonServo.setPosition(0);

        //SET START STATE HERE
        robotState=State.PressBeacon;
    }

    @Override
    public void loop() {
        switch(robotState) {
            case DriveStraight://drive forward for DISTANCE inches
                int DISTANCE=5;
                if(!gyro.isCalibrating()) {//don't move if gyro still calibrating
                    if (resetPosition) {
                        startPosition = right.getCurrentPosition();
                        resetPosition=false;
                    }
                    if (right.getCurrentPosition() < startPosition + methods.Counts(DISTANCE)){
                        right.setPower(.3);
                        left.setPower(.3);
                    } else {
                        right.setPower(0);
                        left.setPower(0);
                        robotState = State.Turn1;
                    }
                }
                break;


            case Turn1://turn right towards the wall DEGREES degrees
                final int DEGREES=45;
                if(Math.abs(gyro.getHeading()-DEGREES)>3){//if heading is more than 3 degrees away from desired position
                    left.setPower(.1);
                    right.setPower(-.1);
                }else{
                    left.setPower(0);
                    right.setPower(0);
                    robotState=State.DriveStraight;
                    resetPosition=true;
                }
                break;


            case DriveToWall://drive DISTANCE inches forward to the wall
                DISTANCE=50;
                if(resetPosition){
                    startPosition=right.getCurrentPosition();
                    resetPosition=false;
                }
                if(right.getCurrentPosition()<startPosition+methods.Counts(DISTANCE)){
                    left.setPower(.5);
                    right.setPower(.5);
                }else{
                    left.setPower(0);
                    right.setPower(0);
                    robotState=State.TurnToBeacon;
                    resetPosition=true;
                }
                break;


            case TurnToBeacon:
                if(Math.abs(gyro.getHeading())>3){
                    right.setPower(-.1);
                    left.setPower(.1);
                }else{
                    right.setPower(0);
                    left.setPower(0);
                    robotState=State.DriveToFirstBeacon;
                }
                break;


            case DriveToBeacon://drive until the robot hits the white line with THRESHOLD RGB DISTANCE_FROM_WALL inches away from wall
                int THRESHOLD=10;
                if (floorSensor.red() < THRESHOLD && floorSensor.green() < THRESHOLD && floorSensor.blue() < THRESHOLD) {
                    //here is where the wall following code goes, maintain DISTANCE_FROM_WALL inches from the wall
                    double MOTOR_POWER=.5;
                    double POWER_DELTA=.1;
                    double rangeSensorDistance=rangeSensor.getDistance(DistanceUnit.INCH);
                    if(rangeSensorDistance>DISTANCE_FROM_WALL){
                        //move closer to the wall, right goes slower
                        right.setPower(MOTOR_POWER-POWER_DELTA);
                        left.setPower(MOTOR_POWER);
                    }else if(rangeSensorDistance<DISTANCE_FROM_WALL){
                        //move further away from the wall, left goes slower
                        right.setPower(MOTOR_POWER);
                        left.setPower(MOTOR_POWER-POWER_DELTA);
                    }else{
                        //drive straight
                        right.setPower(MOTOR_POWER);
                        left.setPower(MOTOR_POWER);
                    }
                }else{
                    left.setPower(0);
                    right.setPower(0);
                    robotState=State.PressBeacon;
                    resetPosition=true;
                }
                break;


            case PressBeacon:
                if(resetPosition){
                    startPosition=right.getCurrentPosition();
                    resetPosition=false;
                }
                if(beaconSensor.red()>beaconSensor.blue()+beaconSensor.green()&&resetButton){//red is more than sum of other channels
                    //button closest is red, press closest button
                    nearButton=true;
                    resetButton=false;
                }else{
                    nearButton=false;
                    resetButton=false;
                }
                if(nearButton){//closest button to the robot
                    buttonServo.setPosition(WHEEL_OUT);//push wheel out
                    if(right.getCurrentPosition()<startPosition+methods.Counts(5)){//drive forward 5 inches
                        right.setPower(.1);
                        left.setPower(.1);
                    }else{//retract servo and move to next beacon
                        beaconsPressed++;
                        buttonServo.setPosition(WHEEL_IN);
                        robotState=State.DriveToBeacon;
                        if(beaconsPressed==2){//intercept driving to next beacon and stop loop if 2 beacons have been pressed
                            robotState=State.Stop;
                        }
                    }
                }else{//furthest button from the robot
                    if(right.getCurrentPosition()<startPosition+methods.Counts(5)){//drive forwards 5 inches
                        right.setPower(.1);
                        left.setPower(.1);
                    }else{//then push wheel out and drive forward 5 inches
                        buttonServo.setPosition(WHEEL_OUT);
                        if(right.getCurrentPosition()<startPosition+methods.Counts(10)){//drive forward another 5 inches(intentionally "Counts(10)")
                            right.setPower(.1);
                            left.setPower(.1);
                        }else{//retract wheel and move to next beacon
                            beaconsPressed++;
                            buttonServo.setPosition(WHEEL_IN);
                            resetPosition=true;
                            robotState=State.DriveToBeacon;
                            if(beaconsPressed==2){//intercept driving to next beacon and stop loop if 2 beacons have been pressed
                                robotState=State.Stop;
                            }
                        }
                    }
                }
                break;

            case Stop:
                //do nothing
                break;
        }
        telemetry.addData("RF",floorSensor.red());
        telemetry.addData("GF",floorSensor.green());
        telemetry.addData("BF",floorSensor.blue());
        telemetry.addData("RB",beaconSensor.red());
        telemetry.addData("GB", beaconSensor.green());
        telemetry.addData("BB", beaconSensor.blue());
        telemetry.addData("left encoder",left.getCurrentPosition());
        telemetry.addData("right encoder",right.getCurrentPosition());
        telemetry.addData("distance",rangeSensor.getDistance(DistanceUnit.INCH));
    }
}
