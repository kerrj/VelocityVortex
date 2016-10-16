package org.firstinspires.ftc.teamcode.Swerve.Core;
;import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * @author Duncan
 * Implementation of 6127V1A360L.5FS absolute encoder for use with swerve drive.
 */
public class AbsoluteEncoder{
    AnalogInput ai;
    double angleOffset;
    boolean flipped = false;

    /**
     *
     * @param angleOffset zero point of encoder in degrees
     * @param ai encoder to be used (analoginput in ftc cdim)
     */
    public AbsoluteEncoder(double angleOffset,AnalogInput ai) {
        this.angleOffset = Math.toRadians(angleOffset);
        this.ai=ai;
    }

    public AbsoluteEncoder(double angleOffset, boolean flipped) {
        this.angleOffset = Math.toRadians(angleOffset);
        this.flipped = flipped;
    }

    /**
     * @return angular position of encoder in radians (0 to 2pi)
     */
    public double getAngle() {
        //convert voltage (0.2-4.8) to radians
        double angle;
        if (flipped) angle = (4.8 - ai.getVoltage()) * (2*Math.PI) / 4.6;
        else angle = (ai.getVoltage() - 0.2) * (2*Math.PI) / 4.6;
        return (angle + angleOffset) % (2*Math.PI);
    }

    /**
     * @return result of getAngle()
     */
    public double pidGet() {
        return getAngle();
    }

}
