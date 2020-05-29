#include "Arduino.h"
#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>
#import "CoreBluetoothWrapper.h"
#import "SERVICES.h"

#define CPP

// An Objective-C class that needs to be accessed from C++
@interface CoreBluetoothWrapped : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>

@property(nonatomic) arduino::Arduino *arduino;
@property(strong, nonatomic) CBCentralManager *centralManager;
@property(strong, nonatomic) CBPeripheral *swagScanner;
@property(strong, nonatomic) CBCharacteristic *rotateTableChar;
@property(strong, nonatomic) NSMutableData *data;
@property(nonatomic, strong) dispatch_queue_t centralQueue;


// constructor override
- (id)init;

// destructor override
- (void)dealloc;


/**
 * Set arduino object as callback object.
 * @param arduino arduino object to make calls on.
 */
- (void)set_rotation_state_callback:(arduino::Arduino *)arduino;

/**
 * Start the bluetooth discovery and initialization process. Will create a CBCentralManager and
 * actively scan for Swag Scanner's bluetooth service. Then it will subscribe to notifications.
 */
- (void)start_bluetooth;

/**
 * Rotate the table with the given angle in degrees.
 * @param deg the angle you want to rotate the table in degrees.
 */
- (void)rotate_table:(int)degrees;

/**
 * Helper method to call setIsRotating on the Arduino.
 * @param dataBytes the bytes received from isRotating characteristic.
 */
- (void)set_arduino_is_rotating:(NSData *)dataBytes;

@end