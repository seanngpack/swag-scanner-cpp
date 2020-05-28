#import <Foundation/Foundation.h>
#import "CoreBluetoothWrapper.h"
#import <CoreBluetooth/CoreBluetooth.h>
#import <Foundation/Foundation.h>
#import "SERVICES.h"

#define CPP

// An Objective-C class that needs to be accessed from C++
@interface CoreBluetoothWrapped : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>

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
 * Start the bluetooth discovery and initialization process. Will create a CBCentralManager and
 * actively scan for Swag Scanner's bluetooth service. Then it will subscribe to notifications.
 */
- (void)start_bluetooth;

/**
 * Rotate the table with the given angle in degrees.
 * @param deg the angle you want to rotate the table in degrees.
 */
- (void)rotate_table:(int) degrees;


@end