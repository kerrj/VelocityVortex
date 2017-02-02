package org.firstinspires.ftc.teamcode.VelocityVortex;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
public class DankMemesAutoBlue extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,Shoot,RotateToFirstBeacon,DriveToSecondBeacon,PressFirstBeacon,PressSecondBeacon,DriveToDefend,Defend,Stop}
    RobotState state=RobotState.DriveForward;
    private double rotateRadius=30;
    boolean waitForServos=true,loopStart=true;
    long startTime;


    @Override
    public void init() {
        super.init();
        initAutonomous();
    }

    public void init_loop(){
        if(!gyro.isCalibrating()) {
            swerveDrive.refreshValues();
            swerveDrive.drive(.4, -1, 0, 0);
            swerveDrive.update(true, 15, true);
        }
    }
    @Override
    public void loop() {
        if(true){
            loopStart=false;
            startTime=System.currentTimeMillis();
        }
        super.loop();
        if(gyro.isCalibrating()){
            return;
        }
        beaconResult=thread.getBeaconResult();

        switch(state){
            case DriveForward:
                waitForServos=false;
                shootRight.setPower(.7);
                shootLeft.setPower(.7);
                if(swerveDrive.getLinearInchesTravelled()>20){
                    shootServo.setPosition(SHOOTER_UP);
                }
                if(driveWithEncoders(.3,-1,0,.5,40)){
                    state=RobotState.PressFirstBeacon;
                    waitForServos=true;
                    shootRight.setPower(0);
                    shootLeft.setPower(0);
                    shootServo.setPosition(SHOOTER_DOWN);
                }
                break;

            case PressFirstBeacon:
                if(alignWithAndPushBeacon("Wheels", beaconResult, Side.BLUE,.25)){
                    state=RobotState.DriveToSecondBeacon;
                }
                break;

            case DriveToSecondBeacon:
                waitForServos=true;
                if(driveWithEncoders(-.4,-1,-.1,.4,30)){
                    state=RobotState.PressSecondBeacon;
                    waitForServos=true;
                }
                break;

            case PressSecondBeacon:
                if(alignWithAndPushBeacon("Legos", beaconResult, Side.BLUE,.2)){
                    state=RobotState.DriveToDefend;
                    buttonWheel.setPosition(WHEEL_IN);
                }
                break;
            case DriveToDefend:
                if(System.currentTimeMillis()-startTime<10000){
                    swerveDrive.drive(-1,-.6,0,0);
                    break;
                }
                waitForServos=false;
                if(driveWithEncoders(-1,-.6,0,.3,30)){
                    state=RobotState.Stop;
                    waitForServos=true;
                }
                break;

            case Stop:
                swerveDrive.drive(0,0,1,0);
                break;
        }

        telemetry.addData("BeaconResult",beaconResult);
        telemetry.addData("Confidence",thread.getConfidence());
        telemetry.addData("State",state);
        swerveDrive.update(waitForServos,30,true);
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
