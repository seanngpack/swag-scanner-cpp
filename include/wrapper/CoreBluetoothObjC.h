#import "CoreBluetoothWrapper.h"
#import <CoreBluetooth/CoreBluetooth.h>
#import <Foundation/Foundation.h>
#import "SERVICES.h"

#define DEFINE_SHARED_INSTANCE_USING_BLOCK(block) \
static dispatch_once_t pred = 0; \
__strong static id _sharedObject = nil; \
dispatch_once(&pred, ^{ \
_sharedObject = block(); \
}); \
return _sharedObject; \

// An Objective-C class that needs to be accessed from C++
@interface MyObject : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>

@property(strong, nonatomic) CBCentralManager *centralManager;
@property(strong, nonatomic) CBPeripheral *swagScanner;
@property(strong, nonatomic) NSMutableData *data;

@property(nonatomic, assign) BOOL keepScanning;

+ (MyObject *)sharedInstance;
// constructor override
- (id)init;
//
// destructor override
- (void)dealloc;

- (void)initialize_bt;

// The Objective-C member function you want to call from C++
- (int)doSomethingWith:(void *)aParameter;
@end