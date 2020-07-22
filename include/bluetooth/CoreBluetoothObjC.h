#include "Arduino.h"
#include "ArduinoEventHandler.h"
#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>
#import "CoreBluetoothWrapper.h"
#import "SERVICES.h"

/**
 * TODO: fix memory leak with holding void pointer references
 */

// An Objective-C class that needs to be accessed from C++
@interface CoreBluetoothWrapped : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>

@property(nonatomic) arduino::Arduino *arduino;
@property(nonatomic) handler::ArduinoEventHandler *arduinoEventHandler;
@property(strong, nonatomic) CBCentralManager *centralManager;
@property(strong, nonatomic) CBPeripheral *swagScanner;
@property(strong, nonatomic) CBCharacteristic *rotateTableChar;
@property(strong, nonatomic) CBCharacteristic *tablePosChar;
@property(strong, nonatomic) NSMutableData *data;
@property(nonatomic, strong) dispatch_queue_t centralQueue;


// constructor override
- (id)init;

// destructor override
- (void)dealloc;


- (void)setHandler:(handler::ArduinoEventHandler *)arduinoEventHandler;

/**
 * Start the bluetooth discovery and initialization process. Will create a CBCentralManager and
 * actively scan for Swag Scanner's bluetooth service. Then it will subscribe to notifications.
 */
- (void)startBluetooth;

/**
 * Rotate the table with the given angle in degrees.
 * @param deg the angle you want to rotate the table in degrees.
 */
- (void)rotateTable:(int)degrees;


/**
 * Helper method to call setIsRotating on the Arduino.
 * @param dataBytes the bytes received from isRotating characteristic.
 */
- (void)setIsRotating:(NSData *)dataBytes;


/**
 * Display whether the table is rotating or not.
 * @param dataBytes bytes from notification.
 */
- (void)displayRotInfo:(NSData *)dataBytes;

/**
 * Display the current position of the table received from the notification.
 * @param dataBytes bytes from notification.
 */
- (void)displayTablePosInfo:(NSData *)dataBytes;

/**
 * Convert bytes to int.
 * @param dataBytes data from characteristic in bytes.
 * @return int.
 */
- (int)bytesToInt:(NSData *)dataBytes;

@end