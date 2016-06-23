package com.mecatronica.arduinoadk;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.ParcelFileDescriptor;
import android.util.Log;

import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;


public class SimpleUsbAccessoryHandler implements UsbAccessoryHandlerInterface {

    // Debug flag
    public static final boolean D = true;
    // TAG is used to debug in Android console
    private static final String TAG = "ArduinoADK";
    private static final String ACTION_USB_PERMISSION = "com.mecatronica.arduinoadk.USB_PERMISSION";

    private UsbManager mUsbManager;
    private UsbAccessory mAccessory;

    private ParcelFileDescriptor mFileDescriptor;
    private FileInputStream mInputStream;
    private FileOutputStream mOutputStream;
    private SerialUsbCommunication communication;
    private Thread commThread;

    /**
     * Check if the current accessory associated with de Intent correspond to the local accessory.
     * @param intent Current Intent object
     * @return true if the accessory is the same, false otherwise
     */
    public boolean matchesThisAccessory(Intent intent)
    {
        if (D)
            Log.d(this.getClass().getSimpleName(), "In matchesThisAccessory mAccessory is: " + mAccessory);

        UsbAccessory accessory = (UsbAccessory) intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
        return (accessory != null) && accessory.equals(mAccessory);
    }

    /**
     * Get accessory permission and open accessory
     * @param intent Current Intent object
     */
    public boolean getPermission(Intent intent) {

        UsbAccessory accessory = (UsbAccessory) intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);

        if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false))
        {
            return openAccessory(accessory);
        }
        else
        {
            if (D)
            {
                Log.d(TAG, "Permission denied for accessory " + accessory);
            }
            return  false;
        }
    }

    public boolean getPermision(UsbAccessory accessory, PendingIntent intent)
    {
        if (!mUsbManager.hasPermission(accessory))
        {
            mUsbManager.requestPermission(accessory, intent);
        }
        return openAccessory(accessory);

    }


    /**
     * Manage connection of accessory
     */
    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver()
    {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this)
                {
                    getPermission(intent);
                }
            }
            // Manage detached
            else if (UsbManager.ACTION_USB_ACCESSORY_DETACHED.equals(action))
            {
                if (matchesThisAccessory(intent))
                    closeAccessory();
            }
        }
    };

    /**
     * Get a BroadcastReceiver that manage connection and detach of accessory.
     * @return BroadcastReceiver that manage connection and detach of accessory.
     */
    public BroadcastReceiver getBroadcastReceiver()
    {
        return mUsbReceiver;
    }


    public String get_ACTION_USB_ACCESSORY_DETACHED()
    {
        return UsbManager.ACTION_USB_ACCESSORY_DETACHED;
    }

    public void setManager(Object manager)
    {
        mUsbManager = (UsbManager) manager;
    }

    public void setAccessory(Object accessoryObj)
    {
        mAccessory = (UsbAccessory) accessoryObj;
    }

    public boolean openAccessory(Object accessoryObj)
    {
        mFileDescriptor = mUsbManager.openAccessory((UsbAccessory) accessoryObj);
        if (mFileDescriptor != null)
        {
            setAccessory(accessoryObj);
            FileDescriptor fd = mFileDescriptor.getFileDescriptor();
            mInputStream = new FileInputStream(fd);
            mOutputStream = new FileOutputStream(fd);

            communication = new SerialUsbCommunication(mInputStream, mOutputStream);
            commThread = new Thread(communication);

            if (D)
                Log.d(TAG, "Accessory opened");
            return true;
        }
        else
        {
            if (D)
                Log.d(TAG, "Accessory open failed");
            return false;
        }
    }

    public boolean ok()
    {
        // Check accessory
        return (mAccessory != null);
    }

    public void closeAccessory() {
        // Cancel any thread currently running a connection
        if (communication != null) {
            communication.cancel();
            communication = null;
            commThread = null;
        }
        // Close input stream
        try
        {
            if (mInputStream != null)
                mInputStream.close();
        }
        catch (Exception ignored) {}
        finally {
            mInputStream = null;
        }
        // Close output stream
        try
        {
            if (mOutputStream != null)
                mOutputStream.close();
        }
        catch (Exception ignored) {}
        finally {
            mOutputStream = null;
        }
        // Close file descriptor and accessory
        try {
            if (mFileDescriptor != null)
                mFileDescriptor.close();
        }
        catch (IOException ignored) {}
        finally {
            mFileDescriptor = null;
            mAccessory = null;
        }
    }

    public boolean hasPermission(Object accessoryObj)
    {
        return mUsbManager.hasPermission((UsbAccessory) accessoryObj);
    }

    public Object getConnectedAccessory()
    {
        UsbAccessory[] accessories = mUsbManager.getAccessoryList();
        UsbAccessory accessory = (accessories == null ? null : accessories[0]);
        return accessory;
    }

    public Object getAccessory()
    {
        return mAccessory;
    }


    public void requestPermission(Object accessoryObj, PendingIntent permissionIntent)
    {
        mUsbManager.requestPermission((UsbAccessory) accessoryObj, permissionIntent);
    }

    public void send(byte data) throws IOException
    {
        if (this.ok())
        {
            communication.send(data);
            if (D)
                Log.d(TAG, "Sending data");
        }
        else
        {
            if (D)
                Log.d(TAG, "No device connected");
        }
    }


}
