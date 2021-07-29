package threadedhardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareThread extends Thread implements Hardware {

    ElapsedTime time;
    double[] lastRun; //Previous run values.
    public Configuration config;
    private volatile boolean stop = false, resetIMU = false;

    private static final Double sync = 0.0;

    public HardwareThread(HardwareMap hwMap, Configuration configuration){
        config = configuration;
        config.Configure(hwMap);
        int size = hardware.size();
        lastRun = new double[size];
        config.setBulkCachingManual(true);
    }

    public void run(){

        time = new ElapsedTime();

        try {
            while(!stop) {
                System.out.println("Hardware cycle: " + time.milliseconds());
                updateCycle(); //Should allow every other thread to simply wait for cycle. Consider moving this or adding a sleep to prevent runValues being off by a cycle.

                readHardware(); //Longest section by a ridiculous margin (about 90% of time).

                runHardware();
            }
        }
        catch(Exception e) {System.out.println(e);}
        finally {
            for(ThreadedHardware d : hardware) {
                d.endThreads();
            }
            updateCycle();
            //Not sure if this will keep values over multiple runs, it should not but I'll need to test.
        }
    }

    private void readHardware(){

        config.clearBulkCache();

        for(ThreadedHardware d : hardware) {
            d.getHardware();
        }
    }

    private void runHardware() {

        for(int i = 0; i < hardware.size(); i++) {
            ThreadedHardware d = hardware.get(i);
            double val;
            if(d instanceof Active && (val = ((Active) d).getRunVal()) != lastRun[i]) {
                ((Active) d).setHardware();
                lastRun[i] = val;
            }
            //instanceof and typecasting allows for sensors to not include setHardware.
        }
    }

    public static void updateCycle() {
        synchronized(sync) {
            sync.notifyAll();
        }
    }

    public static void waitForCycle() {
        synchronized (sync) {
            try {
                sync.wait();
            } catch (Exception e) {
                //I'm using println instead of printStackTrace for all my try-catches, so if you need to trace an error look there.
                System.out.println("Exception " + e + " in HardwareThread.waitForCycle().");
            }
        }
    }

    public void Stop(){
        stop = true;
    }
}
