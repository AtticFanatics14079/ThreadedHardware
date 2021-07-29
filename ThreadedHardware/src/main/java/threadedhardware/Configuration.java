package threadedhardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Configuration extends Hardware {

    void Configure(HardwareMap hwMap);
    void setBulkCachingManual(boolean manual);
    void clearBulkCache();
}
