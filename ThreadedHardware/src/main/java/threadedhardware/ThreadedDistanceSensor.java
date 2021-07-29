package threadedhardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ThreadedDistanceSensor implements Sensor, DistanceSensor {

    private DistanceSensor sensor;
    private int partNum;

    private Thread t = null;

    public volatile boolean gettingInput = true;

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateHardware = true;

    public ThreadedDistanceSensor(HardwareMap hwMap, String objectName) {
        sensor = hwMap.get(DistanceSensor.class, objectName);
        this.partNum = hardware.size();

        hardware.add(this);
    }

    @Override
    public int getPartNum() {
        return partNum;
    }

    @Override
    public double[] get() {
        return hardwareVals;
    }

    @Override
    public void getHardware() {
        hardwareVals = new double[]{sensor.getDistance(DistanceUnit.INCH)};

        updateHardware = !updateHardware;
    }

    @Override
    public void endThreads() {
        //if(t != null && t.isAlive()) t.stop();
        //Extra cycle should take care of this.
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        switch(unit) {
            case INCH: return get()[0];
            case CM: return 2.54 * get()[0];
            case MM: return 25.4 * get()[0];
            case METER: return 0.0254 * get()[0];
        }
        return Double.MAX_VALUE;
    }

    public void pingSensor() {
        Sequence pingSensor = new Sequence(() -> {
            gettingInput = true;
            //Need to add something to wait on at some point
            HardwareThread.waitForCycle();
            gettingInput = false;
        });
        if(t != null && t.isAlive()) return;
        t = new Thread(pingSensor);
        t.start();
    }

    public void retrievingHardware(boolean retrieving) {
        gettingInput = retrieving;
    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
