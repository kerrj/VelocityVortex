package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous_Methods;

/**
 * Created by hunai on 9/23/2016.
 */
@Autonomous
@Disabled
public class RedAutoSixWheel extends OpMode {
    final double WHEEL_IN=0;
    final double WHEEL_OUT=.7;
    final double DISTANCE_FROM_WALL=5;//inches
    ColorSensor floorSensor;
    ColorSensor beaconSensor;
    DeviceInterfaceModule cdim;
    Servo buttonServo;
    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor frontRanger;
    ModernRoboticsI2cRangeSensor rangeMeter;
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
        DriveStraight,Turn1,DriveToWall, TurnToBeacon,PressBeacon,DriveToBeacon,Stop, AlignWithWall
    }

    @Override
    public void init() {
        cdim=hardwareMap.deviceInterfaceModule.get("cdim");
        right=hardwareMap.dcMotor.get("right");
        left=hardwareMap.dcMotor.get("left");
        floorSensor=hardwareMap.colorSensor.get("floorSensor");
        beaconSensor=hardwareMap.colorSensor.get("beaconSensor");
        beaconSensor.enableLed(false);
        buttonServo=hardwareMap.servo.get("buttonServo");
        gyro=hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        I2cDeviceSynch i=hardwareMap.i2cDeviceSynch.get("frontRanger");
        frontRanger=new ModernRoboticsI2cRangeSensor(i);











        I2cDeviceSynch deviceSynch=hardwareMap.i2cDeviceSynch.get("range");
        ModernRoboticsI2cRangeSensor rangeSensor =new ModernRoboticsI2cRangeSensor(deviceSynch);














        I2cAddr front=I2cAddr.create8bit(0x10);
        frontRanger.setI2cAddress(front);
        I2cAddr back=I2cAddr.create8bit(0x28);
        rangeMeter.setI2cAddress(back);
        I2cAddr f=I2cAddr.create8bit(0x42);
        I2cAddr b=I2cAddr.create8bit(0x6c); // must use create8bit to work
        beaconSensor.setI2cAddress(b);
        floorSensor.setI2cAddress(f);
        right.setDirection(DcMotor.Direction.REVERSE);
        floorSensor.enableLed(true);
        buttonServo.setPosition(0);

        //SET START STATE HERE
        robotState=State.DriveStraight;
        beaconsPressed=0;
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
                        right.setPower(.6);
                        left.setPower(.6);
                    } else {
                        right.setPower(0);
                        left.setPower(0);
                        robotState = State.Turn1;
                    }
                }
                break;


            case Turn1://turn right towards the wall DEGREES degrees
                final int DEGREES=45;
                if(Math.abs(gyro.getHeading()-DEGREES)>3){//if heading is more than 8 degrees away from desired position
                    left.setPower(.5);
                    right.setPower(-.5);
                }else{
                    left.setPower(0);
                    right.setPower(0);
                    robotState=State.DriveToWall;
                    resetPosition=true;
                }
                break;


            case DriveToWall://drive DISTANCE inches forward to the wall
                DISTANCE=70;
                if(resetPosition){
                    startPosition=right.getCurrentPosition();
                    resetPosition=false;
                }
                if(right.getCurrentPosition()<startPosition+methods.Counts(DISTANCE)){
                    left.setPower(.7);
                    right.setPower(.7);
                }else{
                    left.setPower(0);
                    right.setPower(0);
                    robotState=State.TurnToBeacon;
                    resetPosition=true;
                }
                break;


            case TurnToBeacon:
                if(Math.abs(gyro.getHeading())>3){
                    right.setPower(.5);
                    left.setPower(-.5);
                }else{
                    right.setPower(0);
                    left.setPower(0);
                    robotState=State.AlignWithWall;
                }
                break;

            case AlignWithWall://move close enough to the wall
                if(frontRanger.getDistance(DistanceUnit.INCH)>5|| rangeMeter.getDistance(DistanceUnit.INCH)>5){
                    //do nothing
                }else{
                    robotState=State.DriveToBeacon;
                }
                break;


            case DriveToBeacon://drive until the robot hits the white line with THRESHOLD RGB DISTANCE_FROM_WALL inches away from wall
                int THRESHOLD=10;
                if (floorSensor.red() < THRESHOLD && floorSensor.green() < THRESHOLD && floorSensor.blue() < THRESHOLD) {
                    //here is where the wall following code goes, maintain DISTANCE_FROM_WALL inches from the wall
                    double MOTOR_POWER=.5;
                    double POWER_DELTA=.4;
                    double frontDistance=frontRanger.getDistance(DistanceUnit.INCH);
                    double backDistance= rangeMeter.getDistance(DistanceUnit.INCH);
                    double distanceDifference=frontDistance-backDistance;
                    if(distanceDifference>.5){//turn to the right
                        right.setPower(-.3);
                        left.setPower(.3);
                    }else if(distanceDifference<-.5){//turn left
                        right.setPower(.3);
                        left.setPower(-.3);
                    }else{
                        //drive straight
                        right.setPower(MOTOR_POWER);
                        left.setPower(MOTOR_POWER);
                    }

//                    if(rangeSensorDistance>DISTANCE_FROM_WALL+.5){
//                        //move closer to the wall, right goes slower
//                        right.setPower(MOTOR_POWER-POWER_DELTA);
//                        left.setPower(MOTOR_POWER);
//                    }else if(rangeSensorDistance<DISTANCE_FROM_WALL-.5){
//                        //move further away from the wall, left goes slower
//                        right.setPower(MOTOR_POWER);
//                        left.setPower(MOTOR_POWER-POWER_DELTA);
//                    }else{
//                        //drive straight
//                        right.setPower(MOTOR_POWER);
//                        left.setPower(MOTOR_POWER);
//                    }

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
                    if(right.getCurrentPosition()<startPosition+methods.Counts(7)){//drive forwards 5 inches
                        right.setPower(.2);
                        left.setPower(.2);
                    }else{//then push wheel out and drive forward 5 inches
                        buttonServo.setPosition(WHEEL_OUT);
                        if(right.getCurrentPosition()<startPosition+methods.Counts(15)){//drive forward another 5 inches(intentionally "Counts(10)")
                            right.setPower(.2);
                            left.setPower(.2);
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
                left.setPower(0);
                right.setPower(0);
                break;
        }
        telemetry.addData("State",robotState.toString());
        telemetry.addData("RF",floorSensor.red());
        telemetry.addData("GF",floorSensor.green());
        telemetry.addData("BF",floorSensor.blue());
        telemetry.addData("RB",beaconSensor.red());
        telemetry.addData("GB", beaconSensor.green());
        telemetry.addData("BB", beaconSensor.blue());
        telemetry.addData("left encoder",left.getCurrentPosition());
        telemetry.addData("right encoder",right.getCurrentPosition());
        telemetry.addData("frontDistance",frontRanger.getDistance(DistanceUnit.INCH));
        telemetry.addData("backDistance", rangeMeter.getDistance(DistanceUnit.INCH));
    }
}
