/*package threadedhardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Use the recommended import (Alt + Enter) to get RoadRunner "Encoder" class. This is the import statement we used, after creating a "Utils" folder.
//import org.firstinspires.ftc.teamcode.Utils.Encoder;

//Uncomment to use EXPLICITLY as RoadRunner odometry pods.
public class ThreadedOdometry implements Sensor, ThreadedHardware {

    private Encoder encoder;
    private int partNum;

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread. Updating it makes the entire array update to main memory.
    private volatile boolean updateHardware = true;

    public ThreadedOdometry(HardwareMap hwMap, String objectName) {
        DcMotorImplEx motor = hwMap.get(DcMotorImplEx.class, objectName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder = new Encoder(motor);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    public void getHardware() {
        hardwareVals = new double[]{encoder.getCurrentPosition(), encoder.getCorrectedVelocity()};
        updateHardware = !updateHardware;
    }

    public void endThreads() {
        //Do nothing
    }

    public void reverse(boolean reverse) {
        encoder.setDirection(reverse ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
    }

    public double getCurrentPosition() {
        return hardwareVals[0];
    }

    public double getCorrectedVelocity() {
        return hardwareVals[1];
    }
}
*/