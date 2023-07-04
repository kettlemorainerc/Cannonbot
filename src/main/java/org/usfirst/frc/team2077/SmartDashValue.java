package org.usfirst.frc.team2077;

import java.util.Optional;

public interface SmartDashValue<ValueType> {
    ValueType get();
    Optional<ValueType> getNullable();
    void set(ValueType to);
}
