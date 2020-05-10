#import "../../include/wrapper/CoreBluetoothObjC.h"
#include <iostream>


// C "trampoline" function to invoke Objective-C method
int MyObjectDoSomethingWith(void *ob, void *aParameter) {
    // Call the Objective-C method using Objective-C syntax
    return [(id) ob doSomethingWith:aParameter];
}

void initialize_bt2(void *myObjectInstance) {
    [(id) myObjectInstance initialize_bt];
}

void *get_wrapper_object() {
    void *obj_ptr = [[MyObject alloc] init];
    return obj_ptr;
}

void *get_shared_instance() {
    return [MyObject sharedInstance];
}

@implementation MyObject
// OBJ C FUNCTIONS HERE
+ (MyObject *)sharedInstance {
    DEFINE_SHARED_INSTANCE_USING_BLOCK(^{
        return [[self alloc] init];
    });
}
- (id)init {
    self = [super init];
    if (self) {
        std::cout << "constructing this bullshit" << std::endl;
        [self initialize_bt];
    }
    return self;
}

- (void)dealloc {
    [super dealloc];
    std::cout << "destructing this bluetooth object";
}


- (void)initialize_bt {
    // Note: creating the manager with initWithDelegate immediately calls centralManagerDidUpdateState
    std::cout << "memory address of central manager before init: ";
    std::cout << _centralManager << std::endl;
    std::cout << "memory address of object: ";
    std::cout << self << std::endl;
    self.centralManager = [[CBCentralManager alloc] initWithDelegate:self queue:nil options:nil];
    std::cout << "memory address of central manager after init: ";
    std::cout << self.centralManager << std::endl;

    std::cout << "central manager created" << std::endl;
    self.data = [[NSMutableData alloc] init];
}


- (int)doSomethingWith:(void *)aParameter {
    // The Objective-C function you wanted to call from C++.
    // do work here..
    int a = *((int *) aParameter);
    std::cout << a << std::endl;


    NSLog(@"%@", ROTATE_TABLE_CHAR_UUID);
    return 21 + a; // half of 42
}

#pragma mark - CBCentralManagerDelegate method

// called immediately after initializing central manager with initWithDelegate.
- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    std::cout << "CALL MEEEW!! called" << std::endl;
    BOOL showAlert = YES;
    NSString *state = @"";
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
            showAlert = NO;
            state = @"Bluetooth LE is turned on and ready for communication.";
            NSLog(@"%@", state);
            self.keepScanning = YES;
            [NSTimer scheduledTimerWithTimeInterval:TIMER_SCAN_INTERVAL target:self selector:@selector(pauseScan) userInfo:nil repeats:NO];
            [self.centralManager scanForPeripheralsWithServices:nil options:nil];
            break;
        case CBManagerStateUnknown:
            state = @"The state of the BLE Manager is unknown.";
            break;
        default:
            state = @"The state of the BLE Manager is unknown.";
    }

    if (showAlert) {
        NSLog(@"Central Manager State: %@", state);
    }

}

// call this during scanning when it finds a peripheral
- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI {
    // Retrieve the peripheral name from the advertisement data using the "kCBAdvDataLocalName" key

    NSString *peripheralName = [advertisementData objectForKey:@"kCBAdvDataLocalName"];
    NSLog(@"NEXT PERIPHERAL: %@ (%@)", peripheralName, peripheral.identifier.UUIDString);
    if (peripheralName) {
        if ([peripheralName isEqualToString:SWAG_SCANNER_NAME]) {
            self.keepScanning = NO;

            // save a reference to the sensor tag
            self.swagScanner = peripheral;
            self.swagScanner.delegate = self;

            // Request a connection to the peripheral
            [self.centralManager connectPeripheral:self.swagScanner options:nil];
        }
    }
}

// called after peripheral is connected
- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral {
    NSLog(@"**** SUCCESSFULLY CONNECTED TO SWAG SCANNER");
    NSLog(@"Connected");

    // Now that we've successfully connected to the SensorTag, let's discover the services.
    // - NOTE:  we pass nil here to request ALL services be discovered.
    //          If there was a subset of services we were interested in, we could pass the UUIDs here.
    //          Doing so saves batter life and saves time.
    [peripheral discoverServices:nil];
}

// called if didDiscoverPeripheral fails to connect
- (void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error {
    NSLog(@"**** CONNECTION FAILED!!!");
}

- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error {
    NSLog(@"**** DISCONNECTED FROM SENSOR TAG!!!");
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
        uint8_t enableValue = 1;
        NSData *enableBytes = [NSData dataWithBytes:&enableValue length:sizeof(uint8_t)];

        // table position
        if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:TABLE_POSITION_CHAR_UUID]]) {
            // Enable Table position notification
            [self.swagScanner setNotifyValue:YES forCharacteristic:characteristic];
        }

        // table rotation
        if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:IS_TABLE_ROTATING_CHAR_UUID]]) {
            // Enable Table rotation notification
            [self.swagScanner setNotifyValue:YES forCharacteristic:characteristic];
        }

    }
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
        }
    }
}

// log the output
- (void)displayInfo:(NSData *)dataBytes {

    // get the data's length - divide by two since we're creating an array that holds 16-bit (two-byte) values...
    NSUInteger dataLength = dataBytes.length / 2;

    // create an array to contain the 16-bit values
    uint16_t dataArray[dataLength];
    for (int i = 0; i < dataLength; i++) {
        dataArray[i] = 0;
    }

    // extract the data from the dataBytes object
    [dataBytes getBytes:&dataArray length:dataLength * sizeof(uint16_t)];
    uint16_t rawValue = dataArray[0];

    NSLog(@"Output from notification: %d", rawValue);
}

@end