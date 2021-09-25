 package threadedhardware;

 import com.qualcomm.robotcore.hardware.AnalogInput;
 import com.qualcomm.robotcore.hardware.AnalogSensor;
 import com.qualcomm.robotcore.hardware.HardwareMap;

 public class ThreadedAnalogSensor implements Sensor, AnalogSensor {

     private AnalogInput sensor;
     private int partNum;

     private InterpretVoltage interpret;

     //Passed into InterpretVoltage to help with calculations, notably for offsetting.
     private double helperValue;

     //Array holding all the hardware inputs.
     private double[] hardwareVals;

     //This variable is here to make sure that hardwareVals is visible to every thread. The value of this variable does not matter.
     private volatile boolean updateHardware = true;

     public ThreadedAnalogSensor(HardwareMap hwMap, String objectName, InterpretVoltage method) {
         sensor = hwMap.get(AnalogInput.class, objectName);
         this.partNum = hardware.size();
         interpret = method;
         hardware.add(this);
     }

     public ThreadedAnalogSensor(HardwareMap hwMap, String objectName, double helperValue, InterpretVoltage method) {
         sensor = hwMap.get(AnalogInput.class, objectName);
         this.partNum = hardware.size();
         interpret = method;
         this.helperValue = helperValue;
         hardware.add(this);
     }

     public interface InterpretVoltage {
         //Does something with the raw voltage ("voltage"), can use the max voltage of the sensor ("maxVoltage") to help.
         double interpret(double voltage, double helperValue);
     }

     @Override
     public double readRawVoltage() {
         return sensor.getVoltage();
     }

     public double getMaxVoltage() {
         return helperValue;
     }

     public double setMaxVoltage() {
         return helperValue;
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
         hardwareVals = new double[]{interpret.interpret(input, helperValue), input};

         updateHardware = !updateHardware;
     }

     @Override
     public void endThreads() {
         //Do nothing
     }
 }
