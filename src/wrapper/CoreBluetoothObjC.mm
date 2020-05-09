#import "../../include/wrapper/CoreBluetoothObjC.h"
#include <iostream>

@implementation MyObject

// C "trampoline" function to invoke Objective-C method
int MyObjectDoSomethingWith(void *ob, void *aParameter) {
    // Call the Objective-C method using Objective-C syntax
    return [(id) ob doSomethingWith:aParameter];
}

// OBJ C FUNCTIONS HERE

void *get_wrapper_object() {
    MyObject *obj_ptr = [[MyObject alloc] init];
    void *obj_ptr_v = obj_ptr;
    return obj_ptr_v;
}

- (void)initialize {
    _centralManager = [[CBCentralManager alloc] initWithDelegate:self queue:nil options:nil];
    _data = [[NSMutableData alloc] init];
}



- (int)doSomethingWith:(void *)aParameter {
    // The Objective-C function you wanted to call from C++.
    // do work here..
    int a = *((int *) aParameter);
    std::cout << a << std::endl;


    NSLog(@"%@", TRANSFER_SERVICE_UUID);
    return 21 + a; // half of 42
}
@end