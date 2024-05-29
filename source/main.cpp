/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"
#include "ble/gap/Gap.h"
#include "ble/gap/AdvertisingDataParser.h"
#include "pretty_printer.h"

const static char PEER_NAME[] = "EnvironmentalSensor";

#define UUID_ENVIRONMENTAL_SERVICE 0x181A
#define UUID_TEMPERATURE_CHAR 0x2A6E
#define UUID_HUMIDITY_CHAR 0x2A6F
#define UUID_PRESSURE_CHAR 0x2A6D

Serial pc(USBTX, USBRX, 14400);

static EventQueue event_queue(/* event count */ 10 * EVENTS_EVENT_SIZE);

static DiscoveredCharacteristic temp_characteristic;
static DiscoveredCharacteristic humidity_characteristic;
static DiscoveredCharacteristic pressure_characteristic;

static bool trigger_temp_characteristic = false;
static bool trigger_humidity_characteristic = false;
static bool trigger_pressure_characteristic = false;


void service_discovery(const DiscoveredService *service) {
    if (service->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
        printf("S UUID-%x attrs[%u %u]\r\n", service->getUUID().getShortUUID(), service->getStartHandle(), service->getEndHandle());
    } else {
        printf("S UUID-");
        const uint8_t *longUUIDBytes = service->getUUID().getBaseUUID();
        for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
            printf("%02x", longUUIDBytes[i]);
        }
        printf(" attrs[%u %u]\r\n", service->getStartHandle(), service->getEndHandle());
    }
}

/* Callback quando la caratteristica viene letta */
void on_characteristic_read(const GattReadCallbackParams *response) {
    if (response->handle == temp_characteristic.getValueHandle()) {
        int16_t temperature;
        memcpy(&temperature, response->data, sizeof(temperature));
        printf("Temperature: %.2f\n", temperature / 100.0);
    } else if (response->handle == humidity_characteristic.getValueHandle()) {
        uint16_t humidity;
        memcpy(&humidity, response->data, sizeof(humidity));
        printf("Humidity: %.2f\n", humidity / 100.0);
    } else if (response->handle == pressure_characteristic.getValueHandle()) {
        uint32_t pressure;
        memcpy(&pressure, response->data, sizeof(pressure));
        printf("Pressure: %.2f\n", pressure / 10.0);
    }
}

/* funzione che legge tutte le caratteristiche */
void read_all_characteristics() {
    if (!BLE::Instance().gattClient().isServiceDiscoveryActive()) {
        if (trigger_temp_characteristic) {
            temp_characteristic.read();
        }
        if (trigger_humidity_characteristic) {
            humidity_characteristic.read();
        }
        if (trigger_pressure_characteristic) {
            pressure_characteristic.read();
        }
    }
}

/* Callback per discovered characteristics */
void characteristic_discovery(const DiscoveredCharacteristic *characteristicP) {
    if (characteristicP->getUUID().getShortUUID() == UUID_TEMPERATURE_CHAR) {
        temp_characteristic = *characteristicP;
        trigger_temp_characteristic = true;
    } else if (characteristicP->getUUID().getShortUUID() == UUID_HUMIDITY_CHAR) {
        humidity_characteristic = *characteristicP;
        trigger_humidity_characteristic = true;
    } else if (characteristicP->getUUID().getShortUUID() == UUID_PRESSURE_CHAR) {
        pressure_characteristic = *characteristicP;
        trigger_pressure_characteristic = true;
    }
}

/* Callback per service discovery termination */
void discovery_termination(Gap::Handle_t connectionHandle) {
    if (trigger_temp_characteristic || trigger_humidity_characteristic || trigger_pressure_characteristic) {
        event_queue.call(read_all_characteristics);
    }
}


/*
void update_temp_characteristic(void) {
    if (!BLE::Instance().gattClient().isServiceDiscoveryActive()) {
        temp_characteristic.read();
        printf("Sta leggendo!!!\n");
        
    }
}

void characteristic_discovery(const DiscoveredCharacteristic *characteristicP) {
    printf("  C UUID-%x valueAttr[%u] props[%x]\r\n", characteristicP->getUUID().getShortUUID(), characteristicP->getValueHandle(), (uint8_t)characteristicP->getProperties().broadcast());
    if (characteristicP->getUUID().getShortUUID() == 0x2A6E) {  !ALERT! Alter this filter to suit your device. 
        temp_characteristic = *characteristicP;
        trigger_temp_characteristic = true;
    }
}

void discovery_termination(Gap::Handle_t connectionHandle) {
    printf("terminated SD for handle %u\r\n", connectionHandle);
    if (trigger_temp_characteristic) {
        trigger_temp_characteristic = false;
        event_queue.call(update_temp_characteristic);
    }
}



void trigger_toggled_write(const GattReadCallbackParams *response) {
    if (response->handle == temp_characteristic.getValueHandle()) {
        printf("trigger_toggled_write: handle %u, offset %u, len %u\r\n", response->handle, response->offset, response->len);
        for (unsigned index = 0; index < response->len; index++) {
            printf("%c[%02x]", response->data[index], response->data[index]);
        }
        printf("\r\n");

        uint8_t toggledValue = response->data[0] ^ 0x1;
        temp_characteristic.write(1, &toggledValue);
    }
}

void trigger_read(const GattWriteCallbackParams *response) {
    if (response->handle == temp_characteristic.getValueHandle()) {
        temp_characteristic.read();
    }
}*/

class Client : ble::Gap::EventHandler {
public:
    Client(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _is_connecting(false) { }

    ~Client() { }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &Client::on_init_complete);

        _event_queue.call_every(500, this, &Client::update_sensor_values);

        _event_queue.dispatch_forever();
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }

        print_mac_address();

    //    _ble.gattClient().onDataRead(trigger_toggled_write);
    //    _ble.gattClient().onDataWritten(trigger_read);
       
        /* Registra la funzione on_characteristic_read come callback per gli eventi di lettura dei dati GATT */
        _ble.gattClient().onDataRead(on_characteristic_read);

        /* Definisce e imposta i parametri di scansione BLE. In questo caso, vengono utilizzati i parametri di default. */ 
        ble::ScanParameters scan_params;
        _ble.gap().setScanParameters(scan_params);
        _ble.gap().startScan();
    }

    void update_sensor_values() {
        if (trigger_temp_characteristic || trigger_humidity_characteristic || trigger_pressure_characteristic) {
            read_all_characteristics();
        }
    }

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startScan();
        _is_connecting = false;
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent& event) {
        if (event.getOwnRole() == ble::connection_role_t::CENTRAL) {
            _ble.gattClient().onServiceDiscoveryTermination(discovery_termination);
            _ble.gattClient().launchServiceDiscovery(
                event.getConnectionHandle(),
                service_discovery,
                characteristic_discovery,
                UUID_ENVIRONMENTAL_SERVICE
             //   UUID_TEMPERATURE_CHAR
            );
        } else {
            _ble.gap().startScan();
        }
        _is_connecting = false;
    }

    void onAdvertisingReport(const ble::AdvertisingReportEvent &event) {
        /* don't bother with analysing scan result if we're already connecting */
        if (_is_connecting) {
            return;
        }

        ble::AdvertisingDataParser adv_data(event.getPayload());

        /* parse the advertising payload, looking for a discoverable device */
        while (adv_data.hasNext()) {
            ble::AdvertisingDataParser::element_t field = adv_data.next();

            /* connect to a discoverable device */
            if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME &&
                field.value.size() == strlen(PEER_NAME) &&
                (memcmp(field.value.data(), PEER_NAME, field.value.size()) == 0)) {

                printf("Adv from: ");
                print_address(event.getPeerAddress().data());
                printf(" rssi: %d, scan response: %u, connectable: %u\r\n",
                       event.getRssi(), event.getType().scan_response(), event.getType().connectable());

                ble_error_t error = _ble.gap().stopScan();

                if (error) {
                    print_error(error, "Error caused by Gap::stopScan");
                    return;
                }

                const ble::ConnectionParameters connection_params;

                error = _ble.gap().connect(
                    event.getPeerAddressType(),
                    event.getPeerAddress(),
                    connection_params
                );

                if (error) {
                    _ble.gap().startScan();
                    return;
                }

                /* we may have already scan events waiting
                 * to be processed so we need to remember
                 * that we are already connecting and ignore them */
                _is_connecting = true;

                return;
            }
        }
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;

    bool _is_connecting;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    pc.printf("inizio");
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    Client demo(ble, event_queue);

    demo.start();

    pc.printf("fine");

    return 0;
}
