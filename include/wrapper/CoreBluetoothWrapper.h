#ifndef __SWAG_SCANNER_COREBLUETOOTHWRAPPER_H__
#define __SWAG_SCANNER_COREBLUETOOTHWRAPPER_H__

/**
 * Get the CoreBluetooth object as a void pointer.
 * @return a void *
 */
void *get_bluetooth_obj();

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
void rotate_table(void *obj, int deg);

#endif
