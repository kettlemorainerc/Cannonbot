package org.usfirst.frc.team2077.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

public class SmartDashString  implements SmartDashValue<String> {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");

    private final NetworkTableEntry entry;
    private List<Runnable> onChange;
    private String value;
    private String key;

    public SmartDashString(String key, String defaultValue, boolean persistent){
        SmartDashboard.putString(key, defaultValue);

        onChange = new LinkedList<Runnable>();

        entry = table.getEntry(key);
        value = defaultValue;
        this.key = key;

        if(persistent) entry.setPersistent();
        else entry.clearPersistent();

        var events = EnumSet.of(
                Kind.kImmediate,
                Kind.kValueAll
        );
        table.addListener(key, events, (networkTable, tableKey, event) -> {
            this.value = event.valueData.value.getString();
//            System.out.println("Updating " + tableKey + ": " + value);
            onChange.forEach(Runnable::run);
        });

    }

    public void onChange(Runnable runnable) {
        this.onChange.add(runnable);
    }

    @Override
    public String get() {
        return value;
    }

    @Override
    public Optional<String> getNullable() { //TODO: ask david what this means
        return Optional.empty();
    }

    @Override
    public void set(String to) {
        if(!Objects.equals(to, value)) entry.setString(to);
    }
}
