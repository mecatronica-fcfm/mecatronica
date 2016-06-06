package com.mecatronica.arduinoadk;


import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbManager;
import android.os.ParcelFileDescriptor;

public interface UsbAccessoryHandlerInterface {

    String get_ACTION_USB_ACCESSORY_DETACHED();

    void setManager(Object manager);

    void setAccessory(Object accessory);

    boolean openAccessory(Object accessory);

    void closeAccessory();

    boolean getPermission(Intent intent);

    boolean matchesThisAccessory(Intent intent);

    boolean hasPermission(Object accessory);

    Object getConnectedAccessory();

    Object getAccessory();

    void requestPermission(Object theAccessoryObj,
                           PendingIntent permissionIntent);
}