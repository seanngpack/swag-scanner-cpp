#ifndef __SWAG_SCANNER_COREBLUETOOTHWRAPPER_H__
#define __SWAG_SCANNER_COREBLUETOOTHWRAPPER_H__

// This is the C "trampoline" function that will be used
// to invoke a specific Objective-C method FROM C++
int MyObjectDoSomethingWith(void *myObjectInstance, void *parameter);

// get object
void * get_object();

#endif
