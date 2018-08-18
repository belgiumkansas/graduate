#include <stdio.h>

#include <stdlib.h>
#include <phidget22.h>

void LocalErrorCatcher(int errorCode);

// -------------------- Event Functions ---------------------------------------


static void CCONV onAttachHandler(PhidgetHandle ph, void *ctx) {
    printf("Phidget attached!\n");
}

void CCONV AttachHandler(PhidgetHandle device) {

	int serialNumber;
	const char *name;

	LocalErrorCatcher(
		Phidget_getDeviceName(NULL, &name));
	LocalErrorCatcher(
		Phidget_getDeviceSerialNumber(NULL, &serialNumber));

	printf("Hello Device %s, Serial Number: %d\n", name, serialNumber);

	return;
}

void CCONV DetachHandler(PhidgetManagerHandle manager, void *userptr, PhidgetHandle device) {

	int serialNumber;
	const char *name;

	LocalErrorCatcher(
		Phidget_getDeviceName(device, &name));
	LocalErrorCatcher(
		Phidget_getDeviceSerialNumber(device, &serialNumber));

	printf("Goodbye Device %s, Serial Number: %d\n", name, serialNumber);

	return;
}

// This error handler can handle any Phidget function that returns an int
void LocalErrorCatcher(int errorCode) {

	const char *errorDescription;
	PhidgetReturnCode ret;

	// If the error code is 0, everything is okay
	if (errorCode != EPHIDGET_OK) {

		// Otherwise, you can print specific messages or perform actions by error value.
		// Here we will simply print the error and exit
		switch (errorCode) {
		default:
			printf("Error: An error occurred with return code %d.\n", errorCode);

			ret = Phidget_getErrorDescription(errorCode, &errorDescription);
			if (ret != EPHIDGET_OK) {
				printf("Phidget_getErrorDescription failed. Exiting...\n");
				exit(1);
			}

			printf("Error Description: %s\n", errorDescription);
			printf("Exiting...\n");
			exit(1);
		}
	}
	return;
}


void CCONV onAccelerationChange(PhidgetAccelerometerHandle ch, void *ctx, const double acceleration[3], double timestamp){
	printf("accel: %f, %f, %f \n", acceleration[0]*9.8, acceleration[1]*9.8, acceleration[2]*9.8);
}

// -------------------- Main Code ---------------------------------------------

int main(int argc, char* argv[]) {

	PhidgetAccelerometerHandle accel;
 	PhidgetAccelerometer_create(&accel);
	Phidget_setDeviceSerialNumber((PhidgetHandle)accel, 297689);
	Phidget_setOnAttachHandler((PhidgetHandle)accel, onAttachHandler, NULL);
	Phidget_openWaitForAttachment((PhidgetHandle)accel, 5000);

	Phidget_setDataInterval((PhidgetHandle)accel, 4);

	PhidgetAccelerometer_setOnAccelerationChangeHandler( accel, onAccelerationChange, NULL);



	printf("Opening...\n");



	printf("Phidget Simple Playground (plug and unplug managers)\n");
	printf("Press Enter to end anytime...\n");
	getchar();

	printf("Closing...\n");

	return 0;
}
