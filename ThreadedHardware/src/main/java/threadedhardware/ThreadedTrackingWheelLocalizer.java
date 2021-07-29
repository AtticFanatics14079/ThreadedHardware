/*package threadedhardware;

import androidx.annotation.NonNull;

//Import statements below are what we used, all of them (and particularly the "Encoder" import) may need to change to match your repo.
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

/*
@Config
//USE THIS CLASS as a tracking wheel localizer for 3 wheel odometry.
public class ThreadedTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer implements Hardware {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.59; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 0.993;
    public static double Y_MULTIPLER = 1.011;

    private ThreadedOdometry leftEncoder, rightEncoder, frontEncoder;

    public ThreadedTrackingWheelLocalizer(HardwareMap hwMap) {

        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new ThreadedOdometry(hwMap, "leftEncoder");
        rightEncoder = new ThreadedOdometry(hwMap, "rightEncoder");
        frontEncoder = new ThreadedOdometry(hwMap, "frontEncoder");

        //leftEncoder.setDirection(Encoder.Direction.REVERSE);
        //rightEncoder.setDirection(Encoder.Direction.REVERSE);
        //frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List a = Arrays.asList(
                encoderTicksToInches(leftEncoder.get()[0] * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.get()[0] * X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.get()[0]) * Y_MULTIPLER);
        return a;
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        //vals.waitForCycle();

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.get()[1]),
                encoderTicksToInches(rightEncoder.get()[1]),
                encoderTicksToInches(frontEncoder.get()[1])
        );
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }
}
*/