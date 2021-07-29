package threadedhardware;

import java.util.ArrayList;

public interface Hardware {

    //Shares the hardware arraylist with everything.
    ArrayList<ThreadedHardware> hardware = new ArrayList<>();
}
