package threadedhardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class SampleConfiguration implements Configuration {

    private List<LynxModule> allHubs;

    public ThreadedMotor backLeft, frontLeft, frontRight, backRight;

    public ThreadedIMU imu;

    public void Configure(HardwareMap hwMap){
        //Create all hardware objects here.

        //For analog sensors, you will need to create and pass a lambda notation that interprets the raw voltage received from the sensor.
        //This is an example, and is what we used for our Maxbotix distance sensors.
        ThreadedAnalogSensor.InterpretVoltage distance = ((double voltage, double max) -> 87.4 * (voltage - 0.138));

        hardware.clear();
        backLeft = new ThreadedMotor(hwMap, "back_left_motor");
        frontLeft = new ThreadedMotor(hwMap, "front_left_motor");
        frontRight = new ThreadedMotor(hwMap, "front_right_motor");
        backRight = new ThreadedMotor(hwMap, "back_right_motor");
        imu = new ThreadedIMU(hwMap); //Defaults to the name "imu", which is default in the hub configuration. There's another constructor that takes a name if your IMU is named differently.
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.reverse(true);
        backRight.reverse(true);

        //Below are other configuration activities that are necessary for writing to file.
        allHubs = hwMap.getAll(LynxModule.class);

        setBulkCachingManual(true);
    }

    public void setBulkCachingManual(boolean manual){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(manual ? LynxModule.BulkCachingMode.MANUAL : LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            if(module.getBulkCachingMode() == LynxModule.BulkCachingMode.MANUAL) {
                module.clearBulkCache();
                module.getBulkData();
            }
        }
    }
}
