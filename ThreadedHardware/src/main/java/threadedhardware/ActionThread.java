package threadedhardware;

//Currently unused, will be used for a future PositionThread, TimeThread, and more.
public interface ActionThread extends Runnable {

    void Stop();
    boolean isAlive();

}
