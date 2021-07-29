package threadedhardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ThreadedServo implements Active {

    private Servo servo;
    private int partNum;

    private ActionThread thread = new NullThread();

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateHardware = true;

    //Value that the servo is set to
    protected volatile double runVal = 0;

    //Constructors

    public ThreadedServo(HardwareMap hwMap, String objectName){
        servo = hwMap.get(Servo.class, objectName);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    //Interface methods

    public void set(double position) {
        runVal = position;
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    public void setHardware() {
        servo.setPosition(runVal);
    }

    @Override
    public double getRunVal() {
        return runVal;
    }

    public void getHardware() {
        hardwareVals = new double[]{servo.getPosition()};
        updateHardware = !updateHardware;
    }

    public void endThreads() {
        thread.Stop();
    }
}
