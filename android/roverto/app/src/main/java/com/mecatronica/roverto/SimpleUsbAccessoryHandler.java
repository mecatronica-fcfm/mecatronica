/**
 * NOT TESTED
 */

package com.mecatronica.roverto;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.ParcelFileDescriptor;
import android.util.Log;

import java.io.FileDescriptor;
import java.io.IOException;


public class SimpleUsbAccessoryHandler {

    // Debug flag
    public static final boolean D = BuildConfig.DEBUG;
    // TAG is used to debug in Android console
    private static final String TAG = "roverto";
    private static final String ACTION_USB_PERMISSION = "com.mecatronica.roverto.USB_PERMISSION";

    // USB Related
    private UsbManager usbManager_;
    private UsbAccessory usbAccessory_;
    private PendingIntent usbPermissionIntent_;
    private ParcelFileDescriptor usbFileDescriptor_;
    // Communication
    private SerialUsbCommunication serialComm_;

    /**
     * Manage connection of accessory
     */
    private BroadcastReceiver mUsbReceiver;


    public SimpleUsbAccessoryHandler(UsbManager manager)
    {
        // Set USB manager
        this.usbManager_ = manager;
    }


    public void close() {
        // Cancel any thread currently running a connection
        if (serialComm_ != null) {
            serialComm_.close();
            serialComm_ = null;
        }
        // Close file descriptor and accessory
        try {
            if (usbFileDescriptor_ != null)
                usbFileDescriptor_.close();
        }
        catch (IOException ignored) {}
        finally {
            usbFileDescriptor_ = null;
            usbAccessory_ = null;
        }
    }

    /**
     * Get a BroadcastReceiver that manage connection and detach of accessory.
     * @return BroadcastReceiver that manage connection and detach of accessory.
     */
    public BroadcastReceiver getBroadcastReceiver(final Activity act)
    {
        if (mUsbReceiver != null) return mUsbReceiver;

        mUsbReceiver = new BroadcastReceiver()
        {
            @Override
            public void onReceive(Context context, Intent intent) {
                Log.d(TAG, "Broadcast receiver");
                String action = intent.getAction();
                // Manage USB permission grandted
                if (ACTION_USB_PERMISSION.equals(action)) {
                    Log.d(TAG, "Permission response");
                    synchronized (act)
                    {
                        UsbAccessory accessory = intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);

                        if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false))
                        {
                            Log.d(TAG, "Permission granted");
                            openAccessory(accessory);
                        }
                        else
                        {
                            Log.w(TAG, "Permission denied for accessory " + accessory);
                        }
                    }
                }
                // Manage detached
                else if (UsbManager.ACTION_USB_ACCESSORY_DETACHED.equals(action))
                {
                    if (matchesThisAccessory(intent))
                        close();
                }
            }
        };
        return mUsbReceiver;
    }

    /**
     * Check if the current accessory associated with de Intent correspond to the local accessory.
     * @param intent Current Intent object
     * @return true if the accessory is the same, false otherwise
     */
    private boolean matchesThisAccessory(Intent intent)
    {
        if (D)
            Log.d(this.getClass().getSimpleName(), "In matchesThisAccessory accessory is: " + usbAccessory_);

        UsbAccessory accessory = (UsbAccessory) intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
        return (accessory != null) && accessory.equals(usbAccessory_);
    }


    /**
     * Connect the USB Accessory
     */
    public void connect()
    {
        if (usbAccessory_ != null) return;

        UsbAccessory accessory = this.getConnectedAccessory();
        if (accessory != null)
        {

            synchronized (mUsbReceiver)
            {
                if (!usbManager_.hasPermission(accessory))
                {
                    Log.d(TAG, "Request permission");
                    usbManager_.requestPermission(accessory, usbPermissionIntent_);
                }
            }
        }
        else
        {
            Log.w(TAG, "Accessory is null");
        }
    }

    public IntentFilter getIntentFilter(Activity act)
    {
        usbPermissionIntent_ = PendingIntent.getBroadcast(act, 0, new Intent(ACTION_USB_PERMISSION), 0);
        IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
        filter.addAction(usbManager_.ACTION_USB_ACCESSORY_DETACHED);
        return filter;
    }

    /**
     * Open a USB Accesory
     * @param accessoryObj
     * @return
     */
    private boolean openAccessory(UsbAccessory accessoryObj)
    {
        Log.d(TAG, "Try to open accessory");
        usbFileDescriptor_ = usbManager_.openAccessory(accessoryObj);
        if (usbFileDescriptor_ != null)
        {
            usbAccessory_ =  accessoryObj;
            FileDescriptor fd = usbFileDescriptor_.getFileDescriptor();
            serialComm_ = new SerialUsbCommunication(fd);

            Log.d(TAG, "Accessory opened");
            return true;
        }
        else
        {
            Log.e(TAG, "Accessory open failed");
            return false;
        }
    }

    public boolean isConnected()
    {
        // Check accessory
        return (usbAccessory_ != null);
    }


    private UsbAccessory getConnectedAccessory()
    {
        UsbAccessory[] accessories = usbManager_.getAccessoryList();
        if (accessories == null) Log.e(TAG, "No device connected!:");
        UsbAccessory accessory = (accessories == null ? null : accessories[0]);
        return accessory;
    }

    /*
    * Communication related methods
    */


    public void write(int data)
    {
        if (this.isConnected())
        {
            serialComm_.write(data);
            if (D)
                Log.d(TAG, "Sending data");
        }
        else
        {
            if (D)
                Log.d(TAG, "No device connected");
        }
    }

    public int read()
    {
        if (this.isConnected())
        {
            int data = serialComm_.read();
            if (D)
                Log.d(TAG, "Reading data");
            return data;
        }
        else
        {
            if (D)
                Log.d(TAG, "No device connected");
        }
        return -1;
    }

    public int available()
    {
        return serialComm_.available();
    }


}
