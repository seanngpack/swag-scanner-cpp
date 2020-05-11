#import "CoreBluetoothWrapper.h"
#import <CoreBluetooth/CoreBluetooth.h>
#import <Foundation/Foundation.h>
#import "SERVICES.h"
#define CPP

// An Objective-C class that needs to be accessed from C++
@interface MyObject : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>

@property(strong, nonatomic) CBCentralManager *centralManager;
@property(strong, nonatomic) CBPeripheral *swagScanner;
@property(strong, nonatomic) NSMutableData *data;

@property(nonatomic, assign) BOOL keepScanning;

// constructor override
- (id)init;

// destructor override
- (void)dealloc;


- (void)printCent;
@end