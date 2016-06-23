package com.mecatronica.arduinoadk;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.PendingIntent;

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

import java.io.IOException;


public class MainActivity extends Activity {
    // Debug flag
    public static final boolean D = BuildConfig.DEBUG;
    // TAG is used to debug in Android console
    private static final String TAG = "ArduinoADK";
    private static final String ACTION_USB_PERMISSION = "com.mecatronica.arduinoadk.USB_PERMISSION";

    SimpleUsbAccessoryHandler usbHandler;

    UsbAccessory mAccessory;
    private UsbManager mUsbManager;
    private PendingIntent mPermissionIntent;
    private boolean mPermissionRequestPending;
    TextView connectionStatus;

    private ToggleButton ledToggleButton;
    private OnCheckedChangeListener ledButtonStateChangeListener = new LedStateChangeListener();

    private class LedStateChangeListener implements OnCheckedChangeListener {

        @Override
        public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            byte msg = (byte) (isChecked ? 1 : 0);
            String toastMsg = (isChecked ? "LED ON" : "LED OFF");
            if (usbHandler != null) {
                try {
                    usbHandler.send(msg);
                }
                catch (IOException e) {
                    if (D)
                        Log.e(TAG, "write failed", e);
                }
            }
            Toast.makeText(MainActivity.this, toastMsg, Toast.LENGTH_SHORT).show();
        }
    }

    private void setConnectionStatus(boolean connected) {
        connectionStatus.setText(connected ? "Connected" : "Disconnected");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        connectionStatus = (TextView) findViewById(R.id.uiConnectionStatus);

        usbHandler = new SimpleUsbAccessoryHandler();
        usbHandler.setManager((UsbManager) getSystemService(USB_SERVICE));

        mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
        IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
        registerReceiver(usbHandler.getBroadcastReceiver(), filter);


        // Get toggle button
        ledToggleButton = (ToggleButton) findViewById(R.id.uiLedButton);
        // Add event
        ledToggleButton.setOnCheckedChangeListener(ledButtonStateChangeListener);

    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        usbHandler.closeAccessory();
        unregisterReceiver(usbHandler.getBroadcastReceiver());
    }

    @Override
    public void onResume() {
        super.onResume();

        if (usbHandler != null && usbHandler.ok())
        {
            setConnectionStatus(true);
            return;
        }

        UsbAccessory accessory = (UsbAccessory) usbHandler.getConnectedAccessory();
        if (accessory != null)
        {
            boolean result = usbHandler.getPermision(accessory, mPermissionIntent);
            setConnectionStatus(result);
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
