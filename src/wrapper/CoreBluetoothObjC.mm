#import "../../include/wrapper/CoreBluetoothObjC.h"


@implementation MyObject

// C "trampoline" function to invoke Objective-C method
int MyObjectDoSomethingWith(void *ob, void *aParameter) {
    // Call the Objective-C method using Objective-C syntax
    return [(id) ob doSomethingWith:aParameter];
}

void * get_object() {
    MyObject *obj_ptr = [[MyObject alloc]init];
    void* obj_ptr_v = obj_ptr;
    return obj_ptr_v;
}

- (int)doSomethingWith:(void *)aParameter {
    // The Objective-C function you wanted to call from C++.
    // do work here..
    return 21; // half of 42
}
@end