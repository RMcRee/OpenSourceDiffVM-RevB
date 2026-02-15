package com.rkm.model;

import com.rkm.Constants;

/**
 * User-configurable scanning parameters.
 * Maps to firmware lines 1041-1057.
 */
public class ScanConfig {
    private final InputChannel[] channels = new InputChannel[Constants.MAX_SCAN_CHANNELS];
    private int count = 1;
    private int integrationCycles = 100;
    private boolean autoZeroEnabled = true;
    private int autoZeroInterval = 10;
    private OutputMode outputMode = OutputMode.Human;

    public ScanConfig() {
        channels[0] = InputChannel.Vx;
    }

    public InputChannel getChannel(int index) { return channels[index]; }
    public void setChannel(int index, InputChannel ch) { channels[index] = ch; }
    public int getCount() { return count; }
    public void setCount(int count) { this.count = count; }
    public int getIntegrationCycles() { return integrationCycles; }
    public void setIntegrationCycles(int cycles) { this.integrationCycles = cycles; }
    public boolean isAutoZeroEnabled() { return autoZeroEnabled; }
    public void setAutoZeroEnabled(boolean enabled) { this.autoZeroEnabled = enabled; }
    public int getAutoZeroInterval() { return autoZeroInterval; }
    public void setAutoZeroInterval(int interval) { this.autoZeroInterval = interval; }
    public OutputMode getOutputMode() { return outputMode; }
    public void setOutputMode(OutputMode mode) { this.outputMode = mode; }

    public boolean addChannel(InputChannel ch) {
        for (int i = 0; i < count; i++) {
            if (channels[i] == ch) return false;  // duplicate
        }
        if (count >= Constants.MAX_SCAN_CHANNELS) return false;
        channels[count++] = ch;
        return true;
    }

    public boolean removeChannel(InputChannel ch) {
        for (int i = 0; i < count; i++) {
            if (channels[i] == ch) {
                System.arraycopy(channels, i + 1, channels, i, count - 1 - i);
                count--;
                return true;
            }
        }
        return false;
    }
}
