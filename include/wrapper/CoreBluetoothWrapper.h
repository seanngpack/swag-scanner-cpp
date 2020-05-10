#ifndef __SWAG_SCANNER_COREBLUETOOTHWRAPPER_H__
#define __SWAG_SCANNER_COREBLUETOOTHWRAPPER_H__

// This is the C "trampoline" function that will be used
// to invoke a specific Objective-C method FROM C++
int MyObjectDoSomethingWith(void *myObjectInstance, void *parameter);

void initialize_bt2(void *myObjectInstance);

// get object
void *get_wrapper_object();

void *get_shared_instance();

#endif
