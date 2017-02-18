package org.firstinspires.ftc.teamcode.VelocityVortex.StateAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
@Disabled
public class RedFast extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,Shoot,RotateToFirstBeacon,DriveToSecondBeacon,PressFirstBeacon,PressSecondBeacon,DriveToDefend,Defend,Stop}
    RobotState state=RobotState.DriveForward;
    private double extraDistance=0;
    boolean waitForServos=true,internalResetPosition=true;
    long bookKeepingTime;


    @Override
    public void init() {
        super.init();
        initAutonomous();
    }

    public void init_loop(){
        if(!gyro.isCalibrating()) {
            swerveDrive.refreshValues();
            swerveDrive.drive(.4, 1, 0, 0);
            swerveDrive.update(true, 15, false);
        }
    }
    @Override
    public void loop() {
        super.loop();
        if(gyro.isCalibrating()){
            return;
        }
        beaconResult=thread.getBeaconResult();

        switch(state){
            case DriveForward:
                waitForServos=false;
                shootRight.setPower(AUTONOMOUS_SHOOTING_POWER);
                shootLeft.setPower(AUTONOMOUS_SHOOTING_POWER);
                if(swerveDrive.getLinearInchesTravelled()>AUTONOMOUS_SHOOT_DRIVE_DISTANCE){
                    shootServo.setPosition(SHOOTER_UP);
                    if(internalResetPosition){
                        bookKeepingTime=System.currentTimeMillis();
                        internalResetPosition=false;
                    }
                    if(System.currentTimeMillis()-bookKeepingTime>SHOOTER_MOVE_TIME){
                        shootServo.setPosition(SHOOTER_DOWN);
                    }
                    if(System.currentTimeMillis()-bookKeepingTime>SHOOTER_MOVE_TIME+SHOOTER_DELAY_TIME){
                        shootServo.setPosition(SHOOTER_UP);
                    }
                }
                if(driveWithEncodersAndGyro(.4, 1, 0, .4, 40)){
                    state=RobotState.PressFirstBeacon;
                    waitForServos=true;
                    shootServo.setPosition(SHOOTER_UP);
                }
                break;

            case PressFirstBeacon:
                if(System.currentTimeMillis()-bookKeepingTime>SHOOTER_MOVE_TIME+SHOOTER_DELAY_TIME){
                    shootServo.setPosition(SHOOTER_UP);
                }
                if(beaconResult== HistogramAnalysisThread.BeaconResult.RED_LEFT){
                    extraDistance=5;
                }
                if(alignWithAndPushBeacon("Tools", beaconResult, Side.RED,.25,1)){
                    state=RobotState.DriveToSecondBeacon;
                    buttonWheel.setPosition(WHEEL_IN);
                    shootLeft.setPower(0);
                    shootRight.setPower(0);
                }
                break;

            case DriveToSecondBeacon:
                waitForServos=true;
                if(driveWithEncodersAndGyro(-.4, 1, .1, .4, 25+extraDistance)){
                    state=RobotState.PressSecondBeacon;
                    waitForServos=true;
                }
                break;

            case PressSecondBeacon:
                buttonWheel.setPosition(WHEEL_OUT);
                if(alignWithAndPushBeacon("Gears", beaconResult, Side.RED,.2,2)){
                    state=RobotState.DriveToDefend;
                    buttonWheel.setPosition(WHEEL_IN);
                }
                break;
            case DriveToDefend:
                waitForServos=false;
                if(driveWithEncodersAndGyro(-1, .2, 0, .3, 50)){
                    state=RobotState.Stop;
                    waitForServos=true;
                }
                break;

            case Stop:
                swerveDrive.drive(0,0,-1,.1);
                break;
        }


        telemetry.addData("BeaconResult",beaconResult);
        telemetry.addData("Confidence",thread.getConfidence());
        telemetry.addData("State",state);
        swerveDrive.update(waitForServos,20,true);
    }

    public void stop(){
        super.stop();
        thread.kill();
        try {
            vuforia.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
