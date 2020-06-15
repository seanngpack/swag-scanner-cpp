#import "../../include/wrapper/CoreBluetoothObjC.h"

#include <iostream>
#include <thread>

/*-------------------------------------------------------
                 C++ Wrapper functions here

---------------------------------------------------------*/
void *get_bluetooth_obj() {
    void *obj_ptr = [[CoreBluetoothWrapped alloc] init];

    std::cout << "object id: " << obj_ptr << std::endl;
    return obj_ptr;
}

void start_bluetooth(void *obj) {
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    [(id) obj performSelectorInBackground:@selector(start_bluetooth) withObject:nil];
    [pool release];
}

void rotate(void *obj, int deg) {
    [(id) obj rotate_table:deg];
}

void set_rotation_state_callback(void *arduino, void *obj) {
    arduino::Arduino *a = static_cast<arduino::Arduino *>(arduino);
    [(id) obj set_rotation_state_callback:a];
}

void set_handler(void *arduino_event_handler, void *obj) {
    handler::ArduinoEventHandler *a = static_cast<handler::ArduinoEventHandler *>(arduino_event_handler);
    [(id) obj set_handler:a];
}



/*-------------------------------------------------------
              Objective-C implementation here

---------------------------------------------------------*/

@implementation CoreBluetoothWrapped

- (void)rotate_table:(int)degrees {
    _arduino->setIsRotating(true);
    NSData *bytes = [NSData dataWithBytes:&degrees length:sizeof(degrees)];
    [_swagScanner
            writeValue:bytes
     forCharacteristic:_rotateTableChar
                  type:CBCharacteristicWriteWithResponse];
}

- (void)set_rotation_state_callback:(arduino::Arduino *)arduino {
    _arduino = arduino;
}

- (void)set_handler:(handler::ArduinoEventHandler *)arduinoEventHandler {
    _arduinoEventHandler = arduinoEventHandler;
    _arduinoEventHandler->print_this();
}


- (id)init {
    self = [super init];
    if (self) {
        return self;
    }
    return self;
}

- (void)start_bluetooth {
    _centralQueue = dispatch_queue_create("centralManagerQueue", DISPATCH_QUEUE_SERIAL);

    @autoreleasepool {
        dispatch_async(_centralQueue, ^{
            self.centralManager = [[CBCentralManager alloc] initWithDelegate:self queue:_centralQueue options:nil];
            NSRunLoop *runLoop = [NSRunLoop currentRunLoop];
            while (([runLoop runMode:NSDefaultRunLoopMode beforeDate:[NSDate distantFuture]])) {
            }
            [[NSRunLoop currentRunLoop] run];
        });
    };
}

- (void)dealloc {
    std::cout << "destructing this bluetooth object";
    [super dealloc];
}


// called immediately after initializing central manager with initWithDelegate.
- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    NSString *state = @"";
    NSDictionary *options = @{CBCentralManagerScanOptionAllowDuplicatesKey: @YES};
    switch ([central state]) {
        case CBManagerStateUnsupported:
            state = @"This device does not support Bluetooth Low Energy.";
            break;
        case CBManagerStateUnauthorized:
            state = @"This app is not authorized to use Bluetooth Low Energy.";
            break;
        case CBManagerStatePoweredOff:
            state = @"Bluetooth on this device is currently powered off.";
            break;
        case CBManagerStateResetting:
            state = @"The BLE Manager is resetting; a state update is pending.";
            break;
        case CBManagerStatePoweredOn:
            state = @"Bluetooth LE is turned on and ready for communication.";
            NSLog(@"%@", state);
            NSLog(@"Scanning for Swag Scanenr now...");

            [_centralManager scanForPeripheralsWithServices:nil
                                                    options:nil];
            break;
        case CBManagerStateUnknown:
            state = @"The state of the BLE Manager is unknown.";
            break;
        default:
            state = @"The state of the BLE Manager is unknown.";
    }
}

// call this during scanning when it finds a peripheral
- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI {
    NSString *peripheralName = advertisementData[@"kCBAdvDataLocalName"];
//    NSLog(@"NEXT PERIPHERAL: %@ (%@)", peripheralName, peripheral.identifier.UUIDString);
//    NSLog(@"NAME: %@ ", peripheral.name);
    if (peripheralName) {
        if ([peripheralName isEqualToString:SWAG_SCANNER_NAME]) {
            self.swagScanner = peripheral;
            self.swagScanner.delegate = self;
            [self.centralManager connectPeripheral:self.swagScanner options:nil];
        }
    }
}

// called after peripheral is connected
- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral {
    [_centralManager stopScan];
    NSLog(@"Scanning stopped");
    NSLog(@"**** SUCCESSFULLY CONNECTED TO SWAG SCANNER");
    NSLog(@"Now looking for services...");
    [peripheral discoverServices:nil];
}

// called if didDiscoverPeripheral fails to connect
- (void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error {
    NSLog(@"**** CONNECTION FAILED!!!");
}

- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error {
    NSLog(@"**** DISCONNECTED FROM SWAG SCANNER");
}

#pragma mark - CBPeripheralDelegate methods

// When the specified services are discovered, the peripheral calls the peripheral:didDiscoverServices: method of its delegate object.
- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error {
    // Core Bluetooth creates an array of CBService objects —- one for each service that is discovered on the peripheral.
    for (CBService *service in peripheral.services) {
        NSLog(@"Discovered service: %@", service);
        if (([service.UUID isEqual:[CBUUID UUIDWithString:UART_SERVICE_UUID]])) {
            [peripheral discoverCharacteristics:nil forService:service];
        }
    }
}

// peripheral's response to discoverCharacteristics
// use this to turn on notifications
- (void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error {
    for (CBCharacteristic *characteristic in service.characteristics) {
        uint8_t enableValue = 0;
        NSData *enableBytes = [NSData dataWithBytes:&enableValue length:sizeof(uint8_t)];

        // rotate table
        if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:ROTATE_TABLE_CHAR_UUID]]) {
            NSLog(@"Enabled table rotation characteristic: %@", characteristic);
            _rotateTableChar = characteristic;
            [self.swagScanner writeValue:enableBytes forCharacteristic:characteristic type:CBCharacteristicWriteWithResponse];
        }

        // table position
        if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:TABLE_POSITION_CHAR_UUID]]) {
            NSLog(@"Enabled table position characteristic with notifications: %@", characteristic);
            [self.swagScanner setNotifyValue:YES forCharacteristic:characteristic];
        }

        // table rotation
        if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:IS_TABLE_ROTATING_CHAR_UUID]]) {
            NSLog(@"Enabled is table rotation? characteristic with notifications: %@", characteristic);
            [self.swagScanner setNotifyValue:YES forCharacteristic:characteristic];
        }
    }
//    _arduino->setIsConnected(true);
//    _arduino->print_this();
    std::unique_lock<std::mutex> ul(_arduinoEventHandler->bt_mutex);
    _arduinoEventHandler->set_is_bt_connected(true);
    ul.unlock();
    _arduinoEventHandler->bt_cv.notify_one();
    ul.lock();
    _arduinoEventHandler->print_this();
}

// start receiving data from this method once we set up notifications. Also can be manually
// called with readValueForCharacteristic
- (void)peripheral:(CBPeripheral *)peripheral didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error {
    if (error) {
        NSLog(@"Error changing notification state: %@", [error localizedDescription]);
    } else {
        // extract the data from the characteristic's value property and display the value based on the characteristic type
        NSData *dataBytes = characteristic.value;
        if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:TABLE_POSITION_CHAR_UUID]]) {
            [self displayInfo:dataBytes];
        } else if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:IS_TABLE_ROTATING_CHAR_UUID]]) {
            [self displayInfo:dataBytes];
            [self set_arduino_is_rotating:dataBytes];
        }
    }
}

- (void)set_arduino_is_rotating:(NSData *)dataBytes {
    int theInteger;
    [dataBytes getBytes:&theInteger length:sizeof(theInteger)];
    if (theInteger == 1) {
        _arduino->setIsRotating(true);
    } else {
        _arduino->setIsRotating(false);
    }
}

// log the output
- (void)displayInfo:(NSData *)dataBytes {
    int theInteger;
    [dataBytes getBytes:&theInteger length:sizeof(theInteger)];
    std::cout << "Output from notification " + std::to_string(theInteger) <<std::endl;

}

@end