package threadedhardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SampleTeleOp extends LinearOpMode {

    HardwareThread hardware;
    SampleConfiguration config;

    Thread t;

    @Override
    public void runOpMode() throws InterruptedException {
        //In the opmode, you MUST start by creating a configuration (i.e. a class implementing the "configuration" interface)
        //and passing that into a new HardwareThread object, then calling .start() on the HardwareThread object. You MUST end with hardware.Stop(),
        //or the hardware thread with continue running even after the op mode stops. I recommend putting a try/catch/finally around the entire
        //opmode, as it should ensure that HardwareThread is stopped even if an error occurs.
        try {
            config = new SampleConfiguration();
            hardware = new HardwareThread(hardwareMap, config);
            hardware.start();
            configureSequences();
            waitForStart();
            //t.start();
            while(!isStopRequested()){
                //Hardware is only updated every cycle, so calling .waitForCycle() in your loop will prevent repeated calculations using the same values.
                hardware.waitForCycle();
                getInput();
            }
        } catch(Exception e) {
            System.out.println("Exception: " + e);
        } finally {
            hardware.Stop();
        }
    }

    //Method to show how sequences are configured.
    private void configureSequences() {
        //In a sequence, you will want to pass a lambda function DESCRIBING THE ACTION YOU WANT TO DO. If this is something you want to do repeatedly
        //(e.g. turn until reaching an angle), you MUST use a while loop.
        Sequence seq = new Sequence(() -> {
            while(Math.abs(config.backLeft.get()[1]) < 1000) {
                setPower(0, 0, 0.5);
                HardwareThread.waitForCycle();
            }
            setPower(0, 0, 0);
        }, () -> {
            //Second lambda function is a CONDITION to move to the next sequence. This is completely optional, but if you would like to
            //move on to a second sequence WITHOUT stopping the current sequence, this will do that ONCE THE THREAD FINISHES (i.e. when loop ends).
            while(Math.abs(config.backLeft.get()[1]) < 200) {
                //Notice how this lambda function does not do anything, it just checks for a condition.
                HardwareThread.waitForCycle();
            };
        }, null);
        //Third variable is the sequence happening BEFORE this one. The sequence passed in (if there is a sequence passed in) will be run prior to this
        //sequence, and if the sequence passed in references ANOTHER sequence, that sequence is first (i.e. if always passing in the sequence created above, sequences
        //run top to bottom in order).
        Sequence seq2 = new Sequence(() -> {
            while(Math.abs(config.backLeft.get()[1]) < 1000) {
                HardwareThread.waitForCycle();
                telemetry.addData("Current position: ", config.backLeft.get()[1]);
                telemetry.update();
            }

        }, seq);

        //Make a thread for the LAST sequence in a chain, and it will run each sequence in order.
        t = new Thread(seq2);

        //Calling .start() on the thread will start the sequence of actions WITHOUT impacting this loop, allowing you to retain driver control and continue looping.
        //When testing this method, the robot should turn until the position of the back left wheel reaches 1000, but only output the wheel's position starting at 200.
    }

    private void getInput(){
        //This could be the main input loop for TeleOp.
        System.out.println("Exited input loop.");
    }

    //Method to set x, y and angular speed on mecanum wheels.
    private void setPower(double px, double py, double pa){
        double p1 = -px + py + pa;
        double p2 = px + py + pa;
        double p3 = -px + py - pa;
        double p4 = px + py - pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        config.backLeft.setPower(p1);
        config.frontLeft.setPower(p2);
        config.frontRight.setPower(p3);
        config.backRight.setPower(p4);
    }
}