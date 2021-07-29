package threadedhardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class ThreadedMotor implements Active, DcMotor {

    DcMotorImplEx motor;
    private int partNum;
    private volatile boolean powerMode = false;

    private ActionThread thread = new NullThread();

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateHardware = true;

    //Value that the motor is set to
    protected volatile double runVal = 0;

    public ThreadedMotor(HardwareMap hwMap, String objectName) {
        motor = hwMap.get(DcMotorImplEx.class, objectName);
        motor.setMode(RunMode.RUN_USING_ENCODER);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    //Constructors



    //Interface methods

    public void set(double velocity) {
        runVal = velocity;
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    public void setHardware() {
        if(powerMode) motor.setPower(runVal);
        else motor.setVelocity(runVal);
    }

    public double getRunVal() {
        return runVal;
    }

    public void getHardware() {
        hardwareVals = new double[]{motor.getVelocity(), (double) motor.getCurrentPosition()};

        updateHardware = !updateHardware;
    }

    public void endThreads() {
        thread.Stop();
    }

    //Class-specific methods

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return null;
    }

    public void setPower(double power) {
        powerMode = true;
        set(power);
    }

    @Override
    public double getPower() {
        return 0;
    }

    //RUN_TO_POSITION not supported, use setTargetPosition instead.
    public void setMode(DcMotor.RunMode mode){
        if(mode != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(mode);
    }

    public void setPowerMode(boolean power) {
        powerMode = power;
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        //Should never be called
    }

    @Deprecated
    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Deprecated
    @Override
    public void setTargetPosition(int position) {
        //Leaving this deprecated, use setPosition instead.
        //Do not call this method.
    }

    @Deprecated
    @Override
    public int getTargetPosition() {
        return -1;
    }

    //isBusy checks for alive threads and non-zero velocity, THIS IS NOT THE SAME AS NORMAL ISBUSY. Not currently in use because the thread is not used.
    @Deprecated
    @Override
    public boolean isBusy() {
        return (thread.isAlive() || get()[0] != 0);
    }

    @Override
    public int getCurrentPosition() {
        return (int) get()[1];
    }

    public void setInternalPID(double... pid) {
        motor.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                pid[0], pid[1], pid[2], pid[3]
        ));
    }

    public void reverse(boolean reverse) {
        motor.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        //Just gonna assume this is either not used or used at the end of an opmode.
        motor.close();
    }
}
