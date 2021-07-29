 package threadedhardware;

 import com.qualcomm.robotcore.hardware.AnalogInput;
 import com.qualcomm.robotcore.hardware.AnalogSensor;
 import com.qualcomm.robotcore.hardware.HardwareMap;

 public class ThreadedAnalogSensor implements Sensor, AnalogSensor {

     private AnalogInput sensor;
     private int partNum;

     private InterpretVoltage interpret;

     private double maxVolt;

     //Array holding all the hardware inputs.
     private double[] hardwareVals;

     //This variable is here to make sure that hardwareVals is visible to every thread. The value of this variable does not matter.
     private volatile boolean updateHardware = true;

     public ThreadedAnalogSensor(HardwareMap hwMap, String objectName, InterpretVoltage method) {
         sensor = hwMap.get(AnalogInput.class, objectName);
         this.partNum = hardware.size();
         interpret = method;
         maxVolt = getMaxVoltage();
         hardware.add(this);
     }

     public ThreadedAnalogSensor(HardwareMap hwMap, String objectName, double maxVoltage, InterpretVoltage method) {
         sensor = hwMap.get(AnalogInput.class, objectName);
         this.partNum = hardware.size();
         interpret = method;
         maxVolt = maxVoltage;
         hardware.add(this);
     }

     public interface InterpretVoltage {
         //Does something with the raw voltage ("voltage"), can use the max voltage of the sensor ("maxVoltage") to help.
         double interpret(double voltage, double maxVoltage);
     }

     @Override
     public double readRawVoltage() {
         return sensor.getVoltage();
     }

     public double getMaxVoltage() {
         return sensor.getMaxVoltage();
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
         double input = sensor.getVoltage();
         hardwareVals = new double[]{interpret.interpret(input, maxVolt), input};

         updateHardware = !updateHardware;
     }

     @Override
     public void endThreads() {
         //Do nothing
     }
 }
