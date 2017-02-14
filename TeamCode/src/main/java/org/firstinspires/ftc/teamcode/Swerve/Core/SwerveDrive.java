package org.firstinspires.ftc.teamcode.Swerve.Core;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.UnsupportedEncodingException;

/**
 * Created by hunai on 9/14/2016.
 * @author duncan
 */

public class SwerveDrive {
    //Add values when swerve is done and also look at what absolute encoder is for
    SwerveModule[] modules;
    double[] angles={0,0,0,0};
    double[] powers={0,0,0,0};
    double[] targetPowers={0,0,0,0};
    int[] positions;
    DcMotor[] motors;
    boolean[] atPositions={false,false,false,false};
    private FTCSwerve ftcSwerve;
    private final int WHEEL_DIAMETER=3;//inches
    private final double GEAR_RATIO=2;
    private final int COUNTS_PER_REV=1120;
    private double deltaCounts=0;
    private double[] lastAcceleration;
    private File directory;
    private File constants;
    double pivotX=0,pivotY=0;
//    private double leftTargetPower=0;
//    private double rightTargetPower=0;

    /**
     * Custom constructor for current robot.
     */
    public SwerveDrive(AnalogInput frontLeft, AnalogInput frontRight, AnalogInput backLeft, AnalogInput backRight,
                       DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb,
                       Servo frontleftServo,Servo frontRightServo, Servo backLeftServo, Servo backRightServo, double width, double length, FTCSwerve ftcSwerve){
        //initialize array of modules
        //array can be any size, as long as the position of each module is specified in its constructor
        directory= FtcRobotControllerActivity.getActivity().getExternalFilesDir(null);
        constants=new File(directory,"constants.txt");
        if(!constants.exists()){
            try {
                constants.createNewFile();
                String contents="{\"lf\":0,\"rf\":0,\"lb\":0,\"rb\":0}";
                FileOutputStream fos=new FileOutputStream(constants);
                byte[] data=contents.getBytes();
                fos.write(data,0,data.length);
                fos.close();
            } catch (IOException e){
                e.printStackTrace();
            }
        }
        try{
            FileInputStream fis=new FileInputStream(constants);
            byte[] data=new byte[fis.available()];
            fis.read(data,0,data.length);
            fis.close();
            String contents=new String(data,"UTF-8");
            JSONObject json=new JSONObject(contents);
            Log.d("json",json.toString());
            modules = new SwerveModule[]{
                    //front left
                    new SwerveModule(lf,frontleftServo, new AbsoluteEncoder(-json.getDouble("lf"), frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModule(rf,frontRightServo, new AbsoluteEncoder(-json.getDouble("rf"), frontRight),width/2,length/2),
                    //back left
                    new SwerveModule(lb,backLeftServo, new AbsoluteEncoder(-json.getDouble("lb"), backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModule(rb,backRightServo, new AbsoluteEncoder(-json.getDouble("rb"), backRight),width/2,-length/2)
            };
            Log.d("lf",Double.toString(json.getDouble("lf")));
        } catch (JSONException e) {
            e.printStackTrace();
            modules = new SwerveModule[] {
                    //front left
                    new SwerveModule(lf,frontleftServo, new AbsoluteEncoder(Constants.FL_OFFSET, frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModule(rf,frontRightServo, new AbsoluteEncoder(Constants.FR_OFFSET, frontRight),width/2,length/2),
                    //back left
                    new SwerveModule(lb,backLeftServo, new AbsoluteEncoder(Constants.BL_OFFSET, backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModule(rb,backRightServo, new AbsoluteEncoder(Constants.BR_OFFSET, backRight),width/2,-length/2)
            };
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
            modules = new SwerveModule[] {
                    //front left
                    new SwerveModule(lf,frontleftServo, new AbsoluteEncoder(Constants.FL_OFFSET, frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModule(rf,frontRightServo, new AbsoluteEncoder(Constants.FR_OFFSET, frontRight),width/2,length/2),
                    //back left
                    new SwerveModule(lb,backLeftServo, new AbsoluteEncoder(Constants.BL_OFFSET, backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModule(rb,backRightServo, new AbsoluteEncoder(Constants.BR_OFFSET, backRight),width/2,-length/2)
            };
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            modules = new SwerveModule[] {
                    //front left
                    new SwerveModule(lf,frontleftServo, new AbsoluteEncoder(Constants.FL_OFFSET, frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModule(rf,frontRightServo, new AbsoluteEncoder(Constants.FR_OFFSET, frontRight),width/2,length/2),
                    //back left
                    new SwerveModule(lb,backLeftServo, new AbsoluteEncoder(Constants.BL_OFFSET, backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModule(rb,backRightServo, new AbsoluteEncoder(Constants.BR_OFFSET, backRight),width/2,-length/2)
            };
        } catch (IOException e) {
            e.printStackTrace();
            modules = new SwerveModule[] {
                    //front left
                    new SwerveModule(lf,frontleftServo, new AbsoluteEncoder(Constants.FL_OFFSET, frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModule(rf,frontRightServo, new AbsoluteEncoder(Constants.FR_OFFSET, frontRight),width/2,length/2),
                    //back left
                    new SwerveModule(lb,backLeftServo, new AbsoluteEncoder(Constants.BL_OFFSET, backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModule(rb,backRightServo, new AbsoluteEncoder(Constants.BR_OFFSET, backRight),width/2,-length/2)
            };
        }
//                modules = new SwerveModule[] {
//                //front left
//                new SwerveModule(lf,frontleftServo, new AbsoluteEncoder(Constants.FL_OFFSET, frontLeft),-width/2,length/2),
//                //front right
//                new SwerveModule(rf,frontRightServo, new AbsoluteEncoder(Constants.FR_OFFSET, frontRight),width/2,length/2),
//                //back left
//                new SwerveModule(lb,backLeftServo, new AbsoluteEncoder(Constants.BL_OFFSET, backLeft),-width/2,-length/2),
//                //back right
//                new SwerveModule(rb,backRightServo, new AbsoluteEncoder(Constants.BR_OFFSET, backRight),width/2,-length/2)
//        };
        motors=new DcMotor[]{lf,rf,lb,rb};
        positions=new int[]{lf.getCurrentPosition(),rf.getCurrentPosition(),lb.getCurrentPosition(),rb.getCurrentPosition()};
        this.ftcSwerve=ftcSwerve;
        lastAcceleration=new double[]{System.nanoTime()/1E6,System.nanoTime()/1E6,System.nanoTime()/1E6,System.nanoTime()/1E6};
    }

    public void setPivotPoint(double x,double y){
        pivotX=x;
        pivotY=y;
    }
    public void driveWithOrient(double translationX, double translationY, double rotation, double heading, double powerScale) {
        Vector[] vects = new Vector[modules.length];
        Vector transVect = new Vector(translationX, translationY),
                pivotVect = new Vector(pivotX, pivotY);
        if(transVect.getMagnitude()>1){
            transVect.normalize();
        }

        double maxDist = 0;
        for (int i = 0; i < modules.length; i++) {
            vects[i] = new Vector(modules[i].positionX, modules[i].positionY);
            vects[i].subtract(pivotVect); //calculate module's position relative to pivot point
            maxDist = Math.max(maxDist, vects[i].getMagnitude()); //find farthest distance from pivot
        }

        double maxPower = 1;
        for (int i = 0; i < modules.length; i++) {
            //rotation motion created by driving each module perpendicular to
            //the vector from the pivot point
            vects[i].makePerpendicular();
            //scale by relative rate and normalize to the farthest module
            //i.e. the farthest module drives with power equal to 'rotation' variable
            vects[i].scale(rotation / maxDist);
            vects[i].add(transVect);
            //calculate largest power assigned to modules
            //if any exceed 100%, all must be scale down
            maxPower = Math.max(maxPower, vects[i].getMagnitude());
        }

        angles[0]=vects[0].getAngle()+Math.PI/2;
        angles[1]=vects[1].getAngle()-Math.PI/2;
        angles[2]=vects[2].getAngle()+Math.PI/2;
        angles[3]=vects[3].getAngle()-Math.PI/2;

        targetPowers[0]=(vects[0].getMagnitude() / maxPower)*powerScale;
        targetPowers[1]=(vects[1].getMagnitude() / maxPower)*powerScale;
        targetPowers[2]=(vects[2].getMagnitude() / maxPower)*powerScale;
        targetPowers[3]=(vects[2].getMagnitude() / maxPower)*powerScale;
    }



//    public void translate(Vector direction,double power){
//        leftTargetPower=power;
//        rightTargetPower=power;
//
//        //left side
//        modules[0].set(direction.getAngle());
//        modules[2].set(direction.getAngle()+Math.PI);//intentionally reversed
//        if(Math.abs(modules[0].getDelta())>Math.PI/2){
//            leftTargetPower*=-1;
//            modules[0].set(direction.getAngle()+Math.PI);
//            modules[2].set(direction.getAngle());//intentionally reversed
//        }
//
//        //right side
//        modules[1].set(direction.getAngle());
//        modules[3].set(direction.getAngle());
//        if(Math.abs(modules[1].getDelta())>Math.PI/2){
//            rightTargetPower*=-1;
//            modules[1].set(direction.getAngle()+Math.PI);
//            modules[3].set(direction.getAngle()+Math.PI);
//        }
//    }
//
//    public void rotate(double power){
//        leftTargetPower=-power;//intentionally negative
//        rightTargetPower=power;
//
//        //left side
//        modules[0].set(Math.PI/4);
//        modules[2].set(3*Math.PI/4);
//        if(Math.abs(modules[0].getDelta())>Math.PI/2){
//            leftTargetPower*=-1;
//            modules[0].set(Math.PI/4+Math.PI);
//            modules[2].set(3*Math.PI/4+Math.PI);
//        }
//
//        //right side
//        modules[1].set(7*Math.PI/4);
//        modules[3].set(5*Math.PI/4);
//        if(Math.abs(modules[1].getDelta())>Math.PI/2){
//            rightTargetPower*=-1;
//            modules[1].set(7*Math.PI/4+Math.PI);
//            modules[3].set(5*Math.PI/4+Math.PI);
//        }
//    }
    public void refreshValues(){
        for(SwerveModule m:modules){
            m.refreshValues();
        }
    }

    /**
     * Method called every loop iteration
     */
    public void update(boolean waitForServos,double threshold, boolean accelerate){
        if(!waitForServos) {
            for(int i=0;i<modules.length;i++){
                SwerveModule module=modules[i];
                if(accelerate){
                    accelerate(i);
                }else{
                    powers[i]=targetPowers[i];
                }
                module.set(angles[i],powers[i]);
                module.update();
                //distance travelled
                double average=0;
                for(int j=0;j<positions.length;j++){
                    int position=motors[j].getCurrentPosition();
                    average += Math.abs(positions[j] -position);
                    positions[j] = position;
                }
                average/=4;
                deltaCounts+=average;
            }
        }else{
            boolean atPosition=true;
            for(int i=0;i<modules.length;i++){
                SwerveModule module=modules[i];
                if(accelerate){
                    accelerate(i);
                }else{
                    powers[i]=targetPowers[i];
                }
                module.set(angles[i],powers[i]);//necessary
                if (Math.abs(module.getDelta())>Math.toRadians(threshold)){
                    atPosition=false;
                    atPositions[i]=false;
                }else{
                    atPositions[i]=true;
                }
            }

            if(atPosition){//if all are at the correct position
                for(int i=0;i<modules.length;i++){
                    SwerveModule module=modules[i];
                    module.update();
                    double average=0;//initialize average
                    for(int j=0;j<positions.length;j++){//iterate over all swerve modules
                        int position=motors[j].getCurrentPosition();
                        average += Math.abs(positions[j] -position);//calculate the change in encoder position
                        //since we last checked and add this to the average
                        positions[j] = position;
                    }
                    average/=4;//divide the average by 4 swerve modules
                    deltaCounts+=average;//add the final change in position to the running total
                }
            }else{
                for(int j=0;j<positions.length;j++){
                    positions[j]=motors[j].getCurrentPosition();
                }
                for(int i=0;i<modules.length;i++){
                    SwerveModule module=modules[i];
                    if(atPositions[i]){
                        module.setPower(0);
                    }else{
                        if(module.getDirection()== SwerveModule.ModuleDirection.clockwise){
                            module.setPower(turnPower);//adjust signs on these depending on motor direction
                        }else{
                            module.setPower(-turnPower);
                        }
                    }
                    module.update();
                }
            }
        }
    }


    private void  accelerate(int i){
        double delta=targetPowers[i]-powers[i];
        double dt=System.nanoTime()/1.0E6-lastAcceleration[i];
        lastAcceleration[i]=System.nanoTime()/1.0E6;

        if(Math.abs(delta)<.05){
            powers[i]=targetPowers[i];
        }else if(delta>0){
            if(powers[i]>0) {
                powers[i] += (dt) / 1500;
            }else{
                powers[i]=targetPowers[i];
            }
        }else if(delta<0){
            if(powers[i]<0) {
                powers[i] -= dt / 1500;
            }else{
                powers[i]=targetPowers[i];
            }
        }

        if(powers[i]>1){
            powers[i]=1;
        }else if(powers[i]<-1){
            powers[i]=-1;
        }
    }


    private double turnPower=.12;

    public void setTurnPower(double turnPower){
        this.turnPower=turnPower;
    }

    public void stop(){
        for(int i=0;i<modules.length;i++){
            powers[i]=0;
            angles[i]=modules[i].steerEncoder.getAngle();
        }
    }

    public void lockWheels(){
        angles[0]=3*Math.PI/4;
        angles[1]=Math.PI/4;
        angles[2]=Math.PI/4;
        angles[3]=3*Math.PI/4;
        for(int i=0;i<modules.length;i++){
            powers[i]=0;
        }
    }

    public double getLinearInchesTravelled(){
        double circumference=Math.PI*WHEEL_DIAMETER;
        double revolutions=deltaCounts/COUNTS_PER_REV;
        double distance=revolutions*circumference*GEAR_RATIO;
        return distance;
    }

    public void resetPosition(){
        deltaCounts=0;
    }

}


