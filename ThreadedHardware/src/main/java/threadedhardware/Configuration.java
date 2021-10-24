package threadedhardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Configuration extends Hardware {

    //Method called by HardwareThread to assign the physical
    //hardware of the robot to certain objects.
    void Configure(HardwareMap hwMap);
    void setBulkCachingManual(boolean manual);
    void clearBulkCache();
}
