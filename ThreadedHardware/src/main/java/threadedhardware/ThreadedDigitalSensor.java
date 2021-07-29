package threadedhardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ThreadedDigitalSensor implements Sensor, DigitalChannel {

    private DigitalChannel sensor;
    private int partNum;

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateVals = true;

    //Maybe add a thread to stop when a condition is met
    private ActionThread thread = new NullThread(); //Not in use at the moment, but likely to be used in future updates.

    //Constructors

    public ThreadedDigitalSensor(HardwareMap hwMap, String objectName) {
        sensor = hwMap.get(DigitalChannel.class, objectName);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    //Interface methods

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    //Binarizes boolean for easier math
    public void getHardware() {
        hardwareVals = new double[]{sensor.getState() ? 1 : 0};

        //Setting a volatile variable saves all values to main memory
        updateVals = !updateVals;
    }

    public void endThreads() {
        //thread.Stop();
    }

    @Override
    public Mode getMode() {
        return null;
    }

    @Override
    public void setMode(Mode mode) {

    }

    @Override
    public boolean getState() {
        return false;
    }

    @Override
    public void setState(boolean state) {

    }

    @Override
    public void setMode(DigitalChannelController.Mode mode) {

    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
