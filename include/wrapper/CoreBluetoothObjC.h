#import "CoreBluetoothWrapper.h"
#import <CoreBluetooth/CoreBluetooth.h>
#import <Foundation/Foundation.h>
#import "SERVICES.h"

// An Objective-C class that needs to be accessed from C++
@interface MyObject : NSObject

@property(strong, nonatomic) CBCentralManager *centralManager;
@property(strong, nonatomic) CBPeripheral *arduinoBLE;
@property(strong, nonatomic) NSMutableData *data;

// The Objective-C member function you want to call from C++
- (int)doSomethingWith:(void *)aParameter;
@end