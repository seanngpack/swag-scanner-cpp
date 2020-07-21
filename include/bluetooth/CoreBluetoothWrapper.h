#ifndef SWAG_SCANNER_COREBLUETOOTHWRAPPER_H
#define SWAG_SCANNER_COREBLUETOOTHWRAPPER_H

/**
 * Get the CoreBluetooth object as a void pointer.
 * @return a void *
 */
void *get_bluetooth_obj();

/**
 * Calls on objective c function to set the callback to the arduino handler object.
 * @param arduino the arduino handler object.
 * @param obj the wrapped object.
 */
void set_handler(void *arduino_event_handler, void *obj);

/**
 * Start the bluetooth discovery and initialization process.
 * @param obj the wrapped object.
 */
void start_bluetooth(void *obj);

/**
 * Rotate the table with the given angle in degrees.
 * @param obj the wrapped object.
 * @param deg the angle you want to rotate the table in degrees.
 */
void rotate(void *obj, int deg);


#endif
