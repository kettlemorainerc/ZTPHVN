package org.usfirst.frc.team2077.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.EnumSet;
import java.util.Objects;
import java.util.Optional;

public class SmartDashNumber implements SmartDashValue<Number> {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");

    private final NetworkTableEntry entry;
    private Number value;

    public SmartDashNumber(String key, Number defaultValue, boolean persistent) {
        if(SmartDashboard.getNumber(key, Double.MIN_VALUE) == Double.MIN_VALUE) {
            SmartDashboard.putNumber(key, defaultValue.doubleValue());
        }

        entry = table.getEntry(key);
        value = defaultValue;

        if(persistent) entry.setPersistent();
        else entry.clearPersistent();

        var events = EnumSet.of(
                Kind.kImmediate,
                Kind.kValueAll
        );
        table.addListener(key, events, (networkTable, tableKey, event) -> {
            this.value = event.valueData.value.getDouble();
            System.out.println("Updating " + tableKey + ": " + value);

        });
    }

    @Override public Number get() {
        return value;
    }

    @Override public Optional<Number> getNullable() {
        return Optional.ofNullable(value);
    }

    @Override public void set(Number to) {
        if(!Objects.equals(to, value)) entry.setNumber(to);
    }
}
