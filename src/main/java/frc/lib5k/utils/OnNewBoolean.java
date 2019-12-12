package frc.lib5k.utils;

/**
 * A boolean that can be checked if it is "new"
 */
public class OnNewBoolean {

    boolean data = false;
    boolean newData = true;

    public OnNewBoolean() {
        this(false);
    }

    public OnNewBoolean(boolean on) {
        set(on);
    }

    public void set(boolean on) {
        data = on;
        newData = true;
    }

    public boolean hasNewData() {
        return newData;
    }

    public boolean read() {
        return data;
    }

    public void markRead() {
        newData = false;
    }
}