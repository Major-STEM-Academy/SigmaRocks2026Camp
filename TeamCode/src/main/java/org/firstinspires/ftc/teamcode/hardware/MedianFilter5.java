package org.firstinspires.ftc.teamcode.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/*
Since the sensor is stuck at its default rate (approx. 33ms), create a Moving Median Filter to work around it.
This mimics the "High Accuracy" mode by throwing out the jittery outliers you've been seeing
 (the 24" vs 27" jumps).
Example: Median Filter (Size 5)
Collect 5 samples from the sensor (this takes ~165ms, similar to a 200ms budget).
Sort them from smallest to largest.
Pick the middle value (the 3rd one).
This completely deletes the "322 inch" spikes and the random jitter, providing a value that is
stable enough to park a 37-inch tall robot.
 */
public class MedianFilter5 {
    private final List<Double> readings = new ArrayList<>();
    private final int windowSize = 5;

    public double update(double newReading) {
        // 1. Add the new reading to our list
        readings.add(newReading);

        // 2. Keep the window size at 5
        if (readings.size() > windowSize) {
            readings.remove(0);
        }

        // 3. If we don't have enough data yet, return the raw reading
        if (readings.size() < windowSize) {
            return newReading;
        }

        // 4. Create a copy to sort (we don't want to mess up the order of the original list)
        List<Double> sortedList = new ArrayList<>(readings);
        Collections.sort(sortedList);

        // 5. Return the middle value (Index 2 is the 3rd item in a list of 5)
        return sortedList.get(2);
    }
}

