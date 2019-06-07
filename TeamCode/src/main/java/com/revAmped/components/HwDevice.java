package com.revAmped.components;

/**
 * Created by zwang on 2/15/2016.
 */
public class HwDevice
{
    protected String id;

    protected HwDevice(String id) {
        this.id = id;
    }

    public String getId() {
        return id;
    }

    public String toString () {
        return id;
    }
}
