package frc.robot;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpeedController implements Sendable {
    private double speed = 0.5;
    DoubleSupplier getSpeed = () -> speed;
    DoubleConsumer setSpeed = (speed) -> this.speed = Math.max(0.0,Math.min(speed,0.5));

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SpeedController");

        builder.addDoubleProperty("speed", getSpeed, setSpeed);

    }

    public void update() {
        SmartDashboard.putNumber("Max Speed", speed);
    }

}
