package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

public class LerpTable {
    private final TreeMap<Double, Double> table = new TreeMap<>();

    public LerpTable(double... pairs) {
        if (pairs.length < 4 || pairs.length % 2 != 0)
            throw new IllegalArgumentException("Invalid pairs");

        for (int i = 0; i < pairs.length; i += 2)
            table.put(pairs[i], pairs[i + 1]);
    }

    public double get(double key) {
        if (table.isEmpty()) throw new IllegalStateException("Table is empty.");

        if (key <= table.firstKey())
            return table.firstEntry().getValue();

        if (key >= table.lastKey())
            return table.lastEntry().getValue();

        Map.Entry<Double, Double> lower = table.floorEntry(key);
        Map.Entry<Double, Double> upper = table.ceilingEntry(key);

        double x1 = lower.getKey();
        double y1 = lower.getValue();
        double x2 = upper.getKey();
        double y2 = upper.getValue();

        double t = (key - x1) / (x2 - x1);

        return y1 + t * (y2 - y1);
    }
}