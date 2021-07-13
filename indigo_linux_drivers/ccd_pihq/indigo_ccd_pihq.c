// All rights reserved.
//
// You can use this software under the terms of 'INDIGO Astronomy
// open-source license' (see LICENSE.md).
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS 'AS IS' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// version history


/** INDIGO Raspberry Pi HQ Camera CCD driver
 \file indigo_ccd_pihq.c
 */

#define DRIVER_VERSION 0x0016
#define DRIVER_NAME "indigo_ccd_pihq"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
#include <sys/time.h>

#include <indigo/indigo_driver_xml.h>

#include "indigo_ccd_pihq.h"

#if !(defined(__APPLE__) && defined(__arm64__))

#include <libusb-1.0/libusb.h>

#define CCD_ADVANCED_GROUP         "Advanced"

#define PRIVATE_DATA               ((pihq_private_data *)device->private_data)

// gp_bits is used as boolean
#define is_connected               gp_bits

// -------------------------------------------------------------------------------- Pi HQ Camera interface implementation

#define us2s(s) ((s) / 1000000.0)
#define s2us(us) ((us) * 1000000)

typedef struct {
	int dev_id;
	int count_open;
	int exp_bin_x, exp_bin_y;
	int exp_frame_width, exp_frame_height;
	int exp_bpp;
	indigo_timer *exposure_timer, *temperature_timer, *guider_timer_ra, *guider_timer_dec;
	double target_temperature, current_temperature;
	long cooler_power;
	bool guide_relays[4];
	unsigned char *buffer;
	long int buffer_size;
	bool can_check_temperature, has_temperature_sensor;
	int gain_highest_dr;
	int offset_highest_dr;
	int gain_unity_gain;
	int offset_unity_gain;
	int gain_lowerst_rn;
	int offset_lowest_rn;
} pihq_private_data;

static indigo_result pihq_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property);

static indigo_result pihq_attach(indigo_device *device) {
	assert(device != NULL);
	assert(PRIVATE_DATA != NULL);
	if (indigo_ccd_attach(device, DRIVER_NAME, DRIVER_VERSION) == INDIGO_OK) {
		// --------------------------------------------------------------------------------
		return indigo_ccd_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

static indigo_result pihq_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property) {
	indigo_result result = INDIGO_OK;
	if ((result = indigo_ccd_enumerate_properties(device, client, property)) == INDIGO_OK) {
		if (IS_CONNECTED) {
		}
	}
	return result;
}

static void pihq_connect_callback(indigo_device *device) {
	if (CONNECTION_CONNECTED_ITEM->sw.value) {
		if (!device->is_connected) { /* Do not double open device */
			if (indigo_try_global_lock(device) != INDIGO_OK) {
				CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
				indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
				indigo_update_property(device, CONNECTION_PROPERTY, "Device is locked");
				return;
			}
			device->is_connected = true;
			CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
		}
	} else {
		if (device->is_connected) {  /* Do not double close device */
			device->is_connected = false;
			indigo_global_unlock(device);
			CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
		}
	}
	indigo_ccd_change_property(device, NULL, CONNECTION_PROPERTY);
	return;
failure:
	indigo_global_unlock(device);
	device->is_connected = false;
	CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
	indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
	indigo_update_property(device, CONNECTION_PROPERTY, NULL);
}

static indigo_result pihq_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	assert(device != NULL);
	assert(DEVICE_CONTEXT != NULL);
	assert(property != NULL);
	if (indigo_property_match(CONNECTION_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CONNECTION
		if (indigo_ignore_connection_change(device, property))
			return INDIGO_OK;
		indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
		CONNECTION_PROPERTY->state = INDIGO_BUSY_STATE;
		indigo_update_property(device, CONNECTION_PROPERTY, NULL);
		indigo_set_timer(device, 0, pihq_connect_callback, NULL);
		return INDIGO_OK;
	}
	return indigo_ccd_change_property(device, client, property);
}

static indigo_result pihq_detach(indigo_device *device) {
	assert(device != NULL);
	if (IS_CONNECTED) {
		indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
		pihq_connect_callback(device);
	}
	INDIGO_DEVICE_DETACH_LOG(DRIVER_NAME, device->name);
	return indigo_ccd_detach(device);
}

static pihq_private_data *private_data = NULL;

static indigo_device *pihq_ccd = NULL;

indigo_result indigo_ccd_pihq(indigo_driver_action action, indigo_driver_info *info) {
	static indigo_device ccd_template = INDIGO_DEVICE_INITIALIZER(
		"Rasppberry Pi HQ Camera",
		pihq_attach,
		pihq_enumerate_properties,
		pihq_change_property,
		NULL,
		pihq_detach
	);
	static indigo_driver_action last_action = INDIGO_DRIVER_SHUTDOWN;

	SET_DRIVER_INFO(info, "Raspberry Pi HQ Camera", __FUNCTION__, DRIVER_VERSION, true, last_action);

	if (action == last_action)
		return INDIGO_OK;

	switch (action) {
		case INDIGO_DRIVER_INIT:
			last_action = action;
			private_data = indigo_safe_malloc(sizeof(pihq_private_data));
			pihq_ccd = indigo_safe_malloc_copy(sizeof(indigo_device), &ccd_template);
			pihq_ccd->private_data = private_data;
			indigo_attach_device(pihq_ccd);
			break;

		case INDIGO_DRIVER_SHUTDOWN:
			VERIFY_NOT_CONNECTED(pihq_ccd);
			last_action = action;
			if (pihq_ccd != NULL) {
				indigo_detach_device(pihq_ccd);
				free(pihq_ccd);
				pihq_ccd = NULL;
			}
			if (private_data != NULL) {
				free(private_data);
				private_data = NULL;
			}
			break;

		case INDIGO_DRIVER_INFO:
			break;
	}

	return INDIGO_OK;
}

#else

indigo_result indigo_ccd_pihq(indigo_driver_action action, indigo_driver_info *info) {
	static indigo_driver_action last_action = INDIGO_DRIVER_SHUTDOWN;

	SET_DRIVER_INFO(info, "Raspberry Pi HQ Camera", __FUNCTION__, DRIVER_VERSION, true, last_action);

	switch(action) {
		case INDIGO_DRIVER_INIT:
		case INDIGO_DRIVER_SHUTDOWN:
			return INDIGO_UNSUPPORTED_ARCH;
		case INDIGO_DRIVER_INFO:
			break;
	}
	return INDIGO_OK;
}

#endif
