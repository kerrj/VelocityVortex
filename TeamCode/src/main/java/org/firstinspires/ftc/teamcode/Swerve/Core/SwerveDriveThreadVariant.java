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

public class SwerveDriveThreadVariant  {
    //Add values when swerve is done and also look at what absolute encoder is for
    SwerveModuleThreadVariant[] modules;
    SwerveThread thread1,thread2;
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
    public boolean waitForServos,accelerate,running=true;
    public int servoThreshold;
    double pivotX=0,pivotY=0;
    //    private double leftTargetPower=0;
    //    private double rightTargetPower=0;

    /**
     * Custom constructor for current robot.
     */
    public SwerveDriveThreadVariant(AnalogInput frontLeft, AnalogInput frontRight, AnalogInput backLeft, AnalogInput backRight,
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
            modules = new SwerveModuleThreadVariant[]{
                    //front left
                    new SwerveModuleThreadVariant(lf,frontleftServo, new AbsoluteEncoder(-json.getDouble("lf"), frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModuleThreadVariant(rf,frontRightServo, new AbsoluteEncoder(-json.getDouble("rf"), frontRight),width/2,length/2),
                    //back left
                    new SwerveModuleThreadVariant(lb,backLeftServo, new AbsoluteEncoder(-json.getDouble("lb"), backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModuleThreadVariant(rb,backRightServo, new AbsoluteEncoder(-json.getDouble("rb"), backRight),width/2,-length/2)
            };
            Log.d("lf",Double.toString(json.getDouble("lf")));
        } catch (JSONException e) {
            e.printStackTrace();
            modules = new SwerveModuleThreadVariant[]{
                    //front left
                    new SwerveModuleThreadVariant(lf,frontleftServo, new AbsoluteEncoder(0, frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModuleThreadVariant(rf,frontRightServo, new AbsoluteEncoder(0, frontRight),width/2,length/2),
                    //back left
                    new SwerveModuleThreadVariant(lb,backLeftServo, new AbsoluteEncoder(0, backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModuleThreadVariant(rb,backRightServo, new AbsoluteEncoder(0, backRight),width/2,-length/2)
            };
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
            modules = new SwerveModuleThreadVariant[]{
                    //front left
                    new SwerveModuleThreadVariant(lf,frontleftServo, new AbsoluteEncoder(0, frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModuleThreadVariant(rf,frontRightServo, new AbsoluteEncoder(0, frontRight),width/2,length/2),
                    //back left
                    new SwerveModuleThreadVariant(lb,backLeftServo, new AbsoluteEncoder(0, backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModuleThreadVariant(rb,backRightServo, new AbsoluteEncoder(0, backRight),width/2,-length/2)
            };
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            modules = new SwerveModuleThreadVariant[]{
                    //front left
                    new SwerveModuleThreadVariant(lf,frontleftServo, new AbsoluteEncoder(0, frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModuleThreadVariant(rf,frontRightServo, new AbsoluteEncoder(0, frontRight),width/2,length/2),
                    //back left
                    new SwerveModuleThreadVariant(lb,backLeftServo, new AbsoluteEncoder(0, backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModuleThreadVariant(rb,backRightServo, new AbsoluteEncoder(0, backRight),width/2,-length/2)
            };
        } catch (IOException e) {
            e.printStackTrace();
            modules = new SwerveModuleThreadVariant[]{
                    //front left
                    new SwerveModuleThreadVariant(lf,frontleftServo, new AbsoluteEncoder(0, frontLeft),-width/2,length/2),
                    //front right
                    new SwerveModuleThreadVariant(rf,frontRightServo, new AbsoluteEncoder(0, frontRight),width/2,length/2),
                    //back left
                    new SwerveModuleThreadVariant(lb,backLeftServo, new AbsoluteEncoder(0, backLeft),-width/2,-length/2),
                    //back right
                    new SwerveModuleThreadVariant(rb,backRightServo, new AbsoluteEncoder(0, backRight),width/2,-length/2)
            };
        }
        motors=new DcMotor[]{lf,rf,lb,rb};
        positions=new int[]{lf.getCurrentPosition(),rf.getCurrentPosition(),lb.getCurrentPosition(),rb.getCurrentPosition()};
        this.ftcSwerve=ftcSwerve;
        lastAcceleration=new double[]{System.nanoTime()/1E6,System.nanoTime()/1E6,System.nanoTime()/1E6,System.nanoTime()/1E6};
        thread1=new SwerveThread(new SwerveModuleThreadVariant[]{modules[0],modules[2]});
        thread2=new SwerveThread(new SwerveModuleThreadVariant[]{modules[1],modules[3]});
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

    /**
     * Method called every loop iteration
     */
    public void update(boolean waitForServos,double threshold, boolean accelerate){
        if(!waitForServos) {
            for(int i=0;i<modules.length;i++){
                SwerveModuleThreadVariant module=modules[i];
                if(accelerate){
                    accelerate(i);
                }else{
                    powers[i]=targetPowers[i];
                }
                module.set(angles[i],powers[i]);
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
                SwerveModuleThreadVariant module=modules[i];
                if (Math.abs(module.getDelta(angles[i]))>Math.toRadians(threshold)){
                    atPosition=false;
                    atPositions[i]=false;
                }else{
                    atPositions[i]=true;
                }
            }

            if(atPosition){//if all are at the correct position
                for(int i=0;i<modules.length;i++){
                    if(accelerate){
                        accelerate(i);
                    }else{
                        powers[i]=targetPowers[i];
                    }
                    modules[i].set(angles[i],powers[i]);
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
                    SwerveModuleThreadVariant module=modules[i];
                    if(atPositions[i]){
                        modules[i].set(angles[i],0);
                    }else{
                        if(module.getDirection(angles[i])== SwerveModuleThreadVariant.ModuleDirection.clockwise){
                            modules[i].set(angles[i],0);//adjust signs on these depending on motor direction
                            modules[i].setMotorPower(turnPower);
                        }else{
                            modules[i].set(angles[i],0);
                            modules[i].setMotorPower(-turnPower);
                        }
                    }
                }
            }
        }
    }
    public void kill(){
        running=false;
        thread1.kill();
        thread2.kill();
        for(SwerveModuleThreadVariant module:modules){
            module.kill();
        }
    }

    private void  accelerate(int i){
        double delta=targetPowers[i]-powers[i];
        double dt=System.nanoTime()/1.0E6-lastAcceleration[i];
        lastAcceleration[i]=System.nanoTime()/1.0E6;

        if(Math.abs(delta)<.03){
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


