package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hunai on 9/14/2016.
 * @author Duncan
 */
public class SwerveModule {
    DcMotor driveMotor; //SpeedController used so this can be talon, victor, jaguar, CAN talon...
    Servo steerServo;
    public AbsoluteEncoder steerEncoder;
    double positionX, positionY; //position of this wheel relative to the center of the robot
    //from the robot's perspective, +y is forward and +x is to the right
    boolean enabled = false;

    /**
     * @param driveMotor motor controller for drive motor
     * @param steerServo servo controller for steer motor
     * @param steerEncoder absolute encoder on steering motor
     * @param positionX x coordinate of wheel relative to center of robot (inches)
     * @param positionY y coordinate of wheel relative to center of robot (inches)
     */
    public SwerveModule(DcMotor driveMotor, Servo steerServo, AbsoluteEncoder steerEncoder, double positionX, double positionY) {
        this.steerServo = steerServo;
        this.driveMotor = driveMotor;
        this.steerEncoder = steerEncoder;
        this.positionX = positionX;
        this.positionY = positionY;
    }
    /**
     * @param angle in radians
     * @param speed motor speed [-1 to 1]
     */
    public void set(double angle, double speed) {
        Servo s;
        if (!enabled) return;
        angle = wrapAngle(angle);
        double dist = Math.abs(angle-steerEncoder.getAngle());
        //if the setpoint is more than 90 degrees from the current position, flip everything
        if (dist > Math.PI/2 && dist < 3*Math.PI/2) {
            angle = wrapAngle(angle + Math.PI);
            speed *= -1;
        }
        steerServo.setPosition(angle);
        driveMotor.setPower(Math.max(-1, Math.min(1, speed))); //coerce speed between -1 and 1
    }
    public void enable() {
        enabled = true;
    }

    public void disable() {
        driveMotor.setPower(0);
        steerServo.setPosition(0);
        enabled = false;
    }

    public void rest() {
        driveMotor.setPower(0);
    }

    private double wrapAngle(double angle) {
        angle %= 2*Math.PI;
        if (angle<0) angle += 2*Math.PI;
        return angle;
    }
}
