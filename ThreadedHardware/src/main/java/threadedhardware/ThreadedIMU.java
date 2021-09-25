package threadedhardware;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class ThreadedIMU implements Sensor, BNO055IMU {

    private BNO055IMU imu;

    public volatile boolean gettingInput = true;

    private double imuOffset = 0;

    private Thread t;

    private int partNum;

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateHardware = true;

    //Defaults to name "imu"
    public ThreadedIMU(HardwareMap hwMap) {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; //Default radians
        imu.initialize(parameters);

        this.partNum = hardware.size();
        System.out.println(partNum);
        hardware.add(this);
    }

    public ThreadedIMU(HardwareMap hwMap, String objectName) {
        imu = hwMap.get(BNO055IMU.class, objectName);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; //Default radians
        initialize(parameters);

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
        if(!gettingInput) return;

        hardwareVals = new double[]{calculateOffset(imu.getAngularOrientation().firstAngle), imu.getAngularOrientation().secondAngle, imu.getAngularOrientation().thirdAngle};

        updateHardware = !updateHardware;
    }

    private double calculateOffset(double input) {
        double val = input - imuOffset;
        val += (val < -Math.PI) ? 2 * Math.PI : 0;
        return val;
    }

    public void pingSensor() {
        Sequence pingSensor = new Sequence(() -> {
            gettingInput = true;
            //Wait on some object
            HardwareThread.waitForCycle();
            gettingInput = false;
        });
        if(t != null && t.isAlive()) return;
        t = new Thread(pingSensor);
        t.start();
    }

    public void endThreads() {
        if(t != null && t.isAlive());
    }

    public void setUnit(BNO055IMU.AngleUnit unit) {
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = unit;
        initialize(param);
    }

    public void resetIMU() {
        if(gettingInput) imuOffset += get()[0];
        else {
            gettingInput = true;
            Sequence delay = new Sequence(() -> {
                try {
                    HardwareThread.waitForCycle();
                    imuOffset += get()[0];
                    gettingInput = false;
                } catch (Exception e) {

                }
            });
            Thread reset = new Thread(delay);
            reset.start();
        }
    }

    public void retrievingHardware(boolean retrieving) {
        gettingInput = retrieving;
    }

    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        return imu.initialize(parameters);
    }

    @NonNull
    @Override
    public Parameters getParameters() {
        return imu.getParameters();
    }

    @Override
    public void close() {
        //Again, unsure when this is used, but most likely it's in cleanup so is fine.
        imu.close();
    }

    //Use get().
    @Deprecated
    @Override
    public Orientation getAngularOrientation() {
        return new Orientation(
                AxesReference.EXTRINSIC, AxesOrder.XYZ, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS, (float) get()[0], (float) get()[1], (float) get()[2], 0
        );
    }

    //Use get().
    @Deprecated
    @Override
    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
        return null;
    }

    //Currently only using heading.
    @Deprecated
    @Override
    public Acceleration getOverallAcceleration() {
        return null;
    }

    //Currently only using heading.
    @Deprecated
    @Override
    public AngularVelocity getAngularVelocity() {
        return null;
    }

    //Currently only using heading.
    @Deprecated
    @Override
    public Acceleration getLinearAcceleration() {
        return null;
    }
    //Currently only using heading.
    @Deprecated
    @Override
    public Acceleration getGravity() {
        return null;
    }

    //Currently only using heading, as the other values seem a bit inaccurate.
    @Deprecated
    @Override
    public Temperature getTemperature() {
        return null;
    }

    //Currently only using heading, as the other values seem a bit inaccurate.
    @Deprecated
    @Override
    public MagneticFlux getMagneticFieldStrength() {
        return null;
    }

    //Currently only using heading, as the other values seem a bit inaccurate.
    @Deprecated
    @Override
    public Quaternion getQuaternionOrientation() {
        return null;
    }

    //Currently only using heading, as the other values seem a bit inaccurate.
    @Deprecated
    @Override
    public Position getPosition() {
        return null;
    }

    //Currently only using heading, as the other values seem a bit inaccurate.
    @Deprecated
    @Override
    public Velocity getVelocity() {
        return null;
    }

    //Currently only using heading, as the other values seem a bit inaccurate.
    @Deprecated
    @Override
    public Acceleration getAcceleration() {
        return null;
    }

    @Override
    public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {
        imu.startAccelerationIntegration(initialPosition, initialVelocity, msPollInterval);
    }

    @Override
    public void stopAccelerationIntegration() {
        imu.stopAccelerationIntegration();
    }

    @Override
    public SystemStatus getSystemStatus() {
        return imu.getSystemStatus();
    }

    @Override
    public SystemError getSystemError() {
        return imu.getSystemError();
    }

    @Override
    public CalibrationStatus getCalibrationStatus() {
        return imu.getCalibrationStatus();
    }

    @Override
    public boolean isSystemCalibrated() {
        return imu.isSystemCalibrated();
    }

    @Override
    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    @Override
    public boolean isAccelerometerCalibrated() {
        return imu.isAccelerometerCalibrated();
    }

    @Override
    public boolean isMagnetometerCalibrated() {
        return imu.isMagnetometerCalibrated();
    }

    @Override
    public CalibrationData readCalibrationData() {
        return imu.readCalibrationData();
    }

    @Override
    public void writeCalibrationData(CalibrationData data) {
        imu.writeCalibrationData(data);
    }

    @Override
    public byte read8(Register register) {
        return imu.read8(register);
    }

    @Override
    public byte[] read(Register register, int cb) {
        return imu.read(register, cb);
    }

    @Override
    public void write8(Register register, int bVal) {
        imu.write8(register, bVal);
    }

    @Override
    public void write(Register register, byte[] data) {
        imu.write(register, data);
    }
}
