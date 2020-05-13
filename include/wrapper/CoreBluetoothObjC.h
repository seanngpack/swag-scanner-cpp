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
@property(strong, nonatomic) NSMutableData *data;
@property(strong) NSTimer *backgroundTimer;
@property(nonatomic, assign) BOOL keepScanning;
@property(nonatomic, strong) dispatch_queue_t centralQueue;


// constructor override
- (id)init;

// destructor override
- (void)dealloc;


- (void)printCent;
@end