package pedropathing;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfiledServo {
    private final Servo servo;
    private double currentPosition;
    private double targetPosition;
    private final double maxSpeed;
    private final boolean reversed;
    private final ElapsedTime timer;
    private final ElapsedTime stableTimer;
    private boolean wasPreviouslyAtTarget = false;

    public MotionProfiledServo(Servo servo, double maxSpeed, boolean reversed) {
        this.servo = servo;
        this.maxSpeed = maxSpeed;
        this.reversed = reversed;
        this.currentPosition = servo.getPosition();
        this.targetPosition = currentPosition;
        this.timer = new ElapsedTime();
        this.stableTimer = new ElapsedTime();
    }

    public void setTargetPosition(double target) {
        this.targetPosition = target;
        this.timer.reset();
        this.stableTimer.reset();
        this.wasPreviouslyAtTarget = false;
    }


    public boolean isAtTarget() {
        return isAtTarget(150); // Defualt delay
    }

    public boolean isAtTarget(long requiredStableMillis) {
        boolean currentlyAtTarget = Math.abs(currentPosition - targetPosition) < 0.005;

        if (currentlyAtTarget) {
            if (!wasPreviouslyAtTarget) {
                stableTimer.reset();
                wasPreviouslyAtTarget = true;
            }
            return stableTimer.milliseconds() >= requiredStableMillis;
        } else {
            wasPreviouslyAtTarget = false;
            return false;
        }
    }

    public void update() {
        double deltaTime = timer.seconds();
        timer.reset();

        double distanceToTarget = targetPosition - currentPosition;
        double absDistance = Math.abs(distanceToTarget);

        if (absDistance < 0.001) {
            currentPosition = targetPosition;
            servo.setPosition(reversed ? 1.0 - currentPosition : currentPosition);
            return;
        }

        double direction = Math.signum(distanceToTarget);
        double maxDelta = maxSpeed * deltaTime;

        if (absDistance <= maxDelta) {
            currentPosition = targetPosition;
        } else {
            currentPosition += direction * maxDelta;
        }

        servo.setPosition(reversed ? 1.0 - currentPosition : currentPosition);
    }
}