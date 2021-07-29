package threadedhardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//IMPORTANT: MotorEncoder should NOT be used as odometry pods for RoadRunner, instead uncomment and use "Encoder".
public class ThreadedMotorEncoder implements Sensor, ThreadedHardware {

    private DcMotorImplEx encoder;
    private int partNum;

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread. Updating it makes the entire array update to main memory.
    private volatile boolean updateHardware = true;

    public ThreadedMotorEncoder(HardwareMap hwMap, String objectName) {
        encoder = hwMap.get(DcMotorImplEx.class, objectName);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        hardwareVals = new double[]{encoder.getCurrentPosition(), encoder.getVelocity()};
        updateHardware = !updateHardware;
    }

    public void endThreads() {
        //Do nothing
    }

    public void reverse(boolean reverse) {
        encoder.setDirection(reverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    public double getCurrentPosition() {
        return hardwareVals[0];
    }

    public double getVelocity() {
        return hardwareVals[1];
    }
}
