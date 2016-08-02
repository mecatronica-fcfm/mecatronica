package com.mecatronica.roverto;

import android.content.ComponentName;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.os.ParcelFileDescriptor;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.PendingIntent;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;

import android.util.Log;

import android.widget.TextView;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;
import android.widget.ToggleButton;
import android.widget.Toast;

import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;

import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

public class MainActivity extends Activity {
    // Debug flag
    public static final boolean D = BuildConfig.DEBUG;
    // TAG is used to debug in Android console
    private static final String TAG = "roverto";
    private static final String ACTION_USB_PERMISSION = "com.mecatronica.arduinoadk.USB_PERMISSION";

    UsbAccessory mAccessory;
    ParcelFileDescriptor mFileDescriptor;
    FileInputStream mInputStream;
    FileOutputStream mOutputStream;
    private UsbManager mUsbManager;
    private PendingIntent mPermissionIntent;
    private boolean mPermissionRequestPending;
    TextView connectionStatus;

    USBCommunication usbCommunication;

    // Socket service
    private RovertoSocketService mRovertoSocketService;
    private boolean mIsBound;

    // Communication bridge
    private CommunicationBridge mCommBridge;

    private ToggleButton ledToggleButton;
    private OnCheckedChangeListener ledButtonStateChangeListener = new LedStateChangeListener();

    private class LedStateChangeListener implements OnCheckedChangeListener {

        @Override
        public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            int cmd = isChecked ? 1 : 0;
            String toastMsg = (isChecked ? "LED ON" : "LED OFF");
            usbCommunication.write(cmd);
            Toast.makeText(MainActivity.this, toastMsg, Toast.LENGTH_SHORT).show();
        }
    }

    /**
     * Manage connection of accesory
     */
    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    // Get accessory
                    UsbAccessory accessory = intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);

                    // Check permission
                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        // Get file descriptors of accesory
                        openAccessory(accessory);
                    } else {
                        if (D)
                            Log.d(TAG, "Permission denied for accessory " + accessory);
                    }
                    mPermissionRequestPending = false;
                }
            }
            // Manage detached
            else if (UsbManager.ACTION_USB_ACCESSORY_DETACHED.equals(action)) {
                UsbAccessory accessory = intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
                if (accessory != null && accessory.equals(mAccessory))
                    closeAccessory();
            }
        }
    };

    private void setConnectionStatus(boolean connected) {
        connectionStatus.setText(connected ? "Connected" : "Disconnected");
    }

    private boolean openAccessory(UsbAccessory accessory) {
        mFileDescriptor = mUsbManager.openAccessory(accessory);
        if (mFileDescriptor != null) {
            mAccessory = accessory;
            FileDescriptor fd = mFileDescriptor.getFileDescriptor();
            mInputStream = new FileInputStream(fd);
            mOutputStream = new FileOutputStream(fd);

            usbCommunication = new USBCommunication();
            usbCommunication.start();

            setConnectionStatus(true);

            mCommBridge = new CommunicationBridge();
            mCommBridge.start();

            if (D)
                Log.d(TAG, "Accessory opened");
            return true;
        } else {
            setConnectionStatus(false);
            if (D)
                Log.d(TAG, "Accessory open failed");
            return false;
        }
    }

    private void closeAccessory() {
        setConnectionStatus(false);

        // Cancel any thread currently running a connection
        if (usbCommunication != null) {
            usbCommunication.cancel();
            usbCommunication = null;
        }

        // Close all streams
        try {
            if (mInputStream != null)
                mInputStream.close();
        } catch (Exception ignored) {
        } finally {
            mInputStream = null;
        }
        try {
            if (mOutputStream != null)
                mOutputStream.close();
        } catch (Exception ignored) {
        } finally {
            mOutputStream = null;
        }
        try {
            if (mFileDescriptor != null)
                mFileDescriptor.close();
        } catch (IOException ignored) {
        } finally {
            mFileDescriptor = null;
            mAccessory = null;
        }
    }

    /**
     * USB communication management
     */
    private class USBCommunication extends Thread {

        static final int MESSAGE_SIZE = RovertoStatus.MESSAGE_SIZE;
        byte[] buffer = new byte[MESSAGE_SIZE];
        BlockingQueue<RovertoStatus> receivedQueue = new LinkedBlockingQueue<RovertoStatus>();
        boolean running;

        USBCommunication() {
            running = true;
        }

        public void run() {
            while (running) {
                try {
                    int bytes = mInputStream.read(buffer);
                    if (bytes > MESSAGE_SIZE - 1) { // The message is 16
                        RovertoStatus msg = RovertoStatus.getFromByteArray(buffer);
                        receivedQueue.put(msg);
                        Log.d(TAG, "Data from USB: " + Arrays.toString(msg.data));
                        Log.d(TAG, "Message queue: " + receivedQueue.size());
                    }
                } catch (Exception ignore) {
                }
            }
        }

        public RovertoStatus read() {
            RovertoStatus msg = null;
            try {
                msg = this.receivedQueue.poll(1, TimeUnit.MILLISECONDS);

                if (msg != null) {
                    Log.d(TAG, "Got message: " + Arrays.toString(msg.data));
                }
            } catch (InterruptedException e) {
                Log.d(TAG, "InterruptedException handling received packet.");
            }
            return msg;
        }

        public void write(int data){
            if (mOutputStream != null) {
                try {
                    mOutputStream.write(data);
                } catch (IOException e) {
                    if (D)
                        Log.e(TAG, "write failed", e);
                }
            }
        }

        public int available() {
            return receivedQueue.size();
        }


        public void cancel() {
            running = false;
        }
    }

    /**
     * Socket communication management
     */
    private ServiceConnection mConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            Log.d(TAG, "Call onServiceConnected");
            RovertoSocketService.LocalBinder binder=(RovertoSocketService.LocalBinder)service;
            mRovertoSocketService=binder.getService();

            if(mRovertoSocketService != null) {
                Log.i(TAG, "Service is bonded successfully!");
            }else{
                Log.e(TAG, "Service null");
            }
        }
        @Override
        public void onServiceDisconnected(ComponentName name) {
            mRovertoSocketService = null;
        }
    };

    private void doBindService() {
        Log.i(TAG, "Call doBindService");
        bindService(new Intent(this, RovertoSocketService.class), mConnection, Context.BIND_AUTO_CREATE);
        mIsBound = true;
    }

    private void doUnbindService() {
        if (mIsBound) {
            // Detach our existing connection.
            unbindService(mConnection);
            mIsBound = false;
        }
    }

    /**
     * Communication bridge between USB and Socket
     */
    class CommunicationBridge extends Thread{
        // Running flag
        private boolean running;

        public CommunicationBridge(){
            running = true;
        }

        public void run(){
            while(running){
                // Check socket service and usb communication
                if (mRovertoSocketService != null && usbCommunication !=null){
                    // Read socket and write to usb
                    if (mRovertoSocketService.available()>0){
                        usbCommunication.write(mRovertoSocketService.read());
                    }
                    // Read data from usb and write socket
                    if (usbCommunication.available()>0){
                        mRovertoSocketService.write(usbCommunication.read());
                    }
                }
            }
        }
        // Stop thread execution
        public void cancel(){
            running = false;
        }
    }


    /**
     * Main activity
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        connectionStatus = (TextView) findViewById(R.id.uiConnectionStatus);

        mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
        IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
        registerReceiver(mUsbReceiver, filter);

        // Get toggle button
        ledToggleButton = (ToggleButton) findViewById(R.id.uiLedButton);
        // Add event
        ledToggleButton.setOnCheckedChangeListener(ledButtonStateChangeListener);

        Log.d(TAG, "Send intent to RovertoSocketService");
        startService(new Intent(this, RovertoSocketService.class));
        doBindService();


    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        doUnbindService();
        closeAccessory();
        unregisterReceiver(mUsbReceiver);
    }

    @Override
    public void onResume() {
        super.onResume();

        if (mAccessory != null) {
            setConnectionStatus(true);
            return;
        }

        UsbAccessory[] accessories = mUsbManager.getAccessoryList();
        UsbAccessory accessory = (accessories == null ? null : accessories[0]);
        if (accessory != null) {
            if (mUsbManager.hasPermission(accessory))
                openAccessory(accessory);
            else {
                setConnectionStatus(false);
                synchronized (mUsbReceiver) {
                    if (!mPermissionRequestPending) {
                        mUsbManager.requestPermission(accessory, mPermissionIntent);
                        mPermissionRequestPending = true;
                    }
                }
            }
        } else {
            setConnectionStatus(false);
            if (D)
                Log.d(TAG, "mAccessory is null");
        }
    }

    @Override
    public void onBackPressed() {
        if (mAccessory != null) {
            new AlertDialog.Builder(this)
                    .setTitle("Closing Activity")
                    .setMessage("Are you sure you want to close this application?")
                    .setPositiveButton("Yes", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            finish();
                        }
                    })
                    .setNegativeButton("No", null)
                    .show();
        } else
            finish();
    }
}