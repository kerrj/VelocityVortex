package org.firstinspires.ftc.teamcode.CameraStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCVuforia;

import java.util.HashMap;

/**
 * Created by Justin on 9/2/2016.
 */

public class SwivelResqMarker extends OpMode {
    private static final double HEAD_MAX=.6;
    private static final double HEAD_MIN=0;
    private FTCVuforia vuforia;
    private Servo head;
    private Servo base;
    @Override
    public void init() {
        head=hardwareMap.servo.get("tilt");
        base=hardwareMap.servo.get("rotate");
        vuforia=new FTCVuforia(FtcRobotControllerActivity.getActivity());
        vuforia.addTrackables("FTCCamera.xml");
        vuforia.initVuforia();
        head.setPosition(HEAD_MIN+.1);
        base.setPosition(.5);
    }

    @Override
    public void loop() {
        HashMap<String, double[]> data=vuforia.getVuforiaData();
        if(data.containsKey("resq")) {
            try{
                telemetry.addData("data", "ok");
                telemetry.addData("rotationx", data.get("resq")[0]);
                telemetry.addData("rotationy", data.get("resq")[1]);
                telemetry.addData("rotationz", data.get("resq")[2]);
                telemetry.addData("distancex", data.get("resq")[3]);
                telemetry.addData("distancey", data.get("resq")[4]);
                telemetry.addData("distancez", data.get("resq")[5]);
                double yangle=180/Math.PI*Math.atan2(data.get("resq")[4],data.get("resq")[5]);
                telemetry.addData("yangle",yangle);
                double xangle=180/Math.PI*Math.atan2(data.get("resq")[3],data.get("resq")[5]);
                telemetry.addData("xangle",xangle);
                moveBase(yangle);
                moveHead(xangle);
            }catch(NullPointerException n){
                n.printStackTrace();
                telemetry.addData("data","null");
            }
        }else{
            telemetry.addData("data","null");
        }
    }
    public void moveHead(double degrees){
        double position=head.getPosition()+(degreesToPostion(degrees)-.5)/10;
        if(position>HEAD_MIN&&position<HEAD_MAX){
            head.setPosition(position);
        }else{
            if(position<HEAD_MIN){
                head.setPosition(HEAD_MIN);
            }
            if(position>HEAD_MAX){
                head.setPosition(HEAD_MAX);
            }
        }
    }
    public void moveBase(double degrees){
        double position=base.getPosition()-(degreesToPostion(degrees)-.5)/10;
        if(position>0&&position<1){
            base.setPosition(position);
        }else{
            if(position<0){
                base.setPosition(0);
            }
            if(position>1){
                base.setPosition(1);
            }
        }
    }
    public double degreesToPostion(double degrees){
        if(degrees>-90&&degrees<90){
            double position=degrees/180+.5;
            return position;
        }else{
            return .5;
        }
    }
    @Override
    public void stop(){
        super.stop();
        try {
            vuforia.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
