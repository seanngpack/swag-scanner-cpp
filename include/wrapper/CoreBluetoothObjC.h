#import <Foundation/Foundation.h>
#import "CoreBluetoothWrapper.h"
#import <CoreBluetooth/CoreBluetooth.h>
#import <Foundation/Foundation.h>
#import "SERVICES.h"

#define CPP

// An Objective-C class that needs to be accessed from C++
@interface MyObject : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>

@property(strong, nonatomic) CBCentralManager *centralManager;
@property(strong, nonatomic) CBPeripheral *swagScanner;
@property(strong, nonatomic) CBCharacteristic *rotateTableChar;
@property(strong, nonatomic) NSMutableData *data;
@property(nonatomic, strong) dispatch_queue_t centralQueue;


// constructor override
- (id)init;

// destructor override
- (void)dealloc;

- (void)start_bt;

- (void)rotate_table:(int) degrees;


@end