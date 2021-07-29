package threadedhardware;

public class Sequence implements Runnable {

    private volatile Action action;
    private volatile Condition condition = null;
    private Sequence sequence;

    private volatile boolean conditionFinished = false;

    public interface Action extends Runnable{
        void run();
    }

    //Condition should loop UNTIL A CONDITION IS MET.
    public interface Condition extends Runnable {
        void run();
    }

    public Sequence(Action action, Condition condition, Sequence sequence){
        this.action = action;
        this.condition = condition;
        this.sequence = sequence;
    }

    public Sequence(Action action, Sequence sequence){
        this.action = action;
        this.sequence = sequence;
    }

    public Sequence(Action action){
        this.action = action;
    }

    public void run(){
        if(sequence != null) {
            sequence.run();
        }

        //Runs condition first
        Thread actionThread = new Thread(action);
        Thread conditionThread = null;
        if(condition != null) {
            conditionThread = new Thread(condition);
            conditionThread.start();
            actionThread.start();
        } else {
            actionThread.run();
        }
        while((conditionThread == null || conditionThread.isAlive()) && actionThread.isAlive()){
            HardwareThread.waitForCycle();
        }
    }
}
