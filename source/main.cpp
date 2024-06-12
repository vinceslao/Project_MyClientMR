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

#include <cstdio>
#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"
#include "ble/gap/Gap.h"
#include "ble/gap/AdvertisingDataParser.h"
#include "pretty_printer.h"

const static char PEER_NAME[] = "EnvironmentalSensor";
const static char PEER2_NAME[] = "RGBSensor";

#define UUID_ENVIRONMENTAL_SERVICE 0x181A
#define UUID_TEMPERATURE_CHAR 0x2A6E
#define UUID_HUMIDITY_CHAR 0x2A6F
#define UUID_PRESSURE_CHAR 0x2A6D

// UUID RGB
#define UUID_RGB_SERVICE "12345678-1234-5678-1234-56789abcdef0"
#define UUID_RED_CHAR "12345678-1234-5678-1234-56789abcdef1"
#define UUID_GREEN_CHAR "12345678-1234-5678-1234-56789abcdef2"
#define UUID_BLUE_CHAR "12345678-1234-5678-1234-56789abcdef3"

const uint8_t ENV_MAC_ADDRESS[] = {0xF0, 0x6A, 0x41, 0xDD, 0x3F, 0x8B};
const uint8_t RGB_MAC_ADDRESS[] = {0xF7, 0xD9, 0xEB, 0x6B, 0x1A, 0x7F};


Serial pc(USBTX, USBRX, 14400);

static EventQueue event_queue(/* event count */ 10 * EVENTS_EVENT_SIZE);

static DiscoveredCharacteristic temp_characteristic;
static DiscoveredCharacteristic humidity_characteristic;
static DiscoveredCharacteristic pressure_characteristic;

static DiscoveredCharacteristic red_characteristic;
static DiscoveredCharacteristic green_characteristic;
static DiscoveredCharacteristic blue_characteristic;

static bool trigger_temp_characteristic = false;
static bool trigger_humidity_characteristic = false;
static bool trigger_pressure_characteristic = false;

static bool trigger_rgb_red_characteristic = false;
static bool trigger_rgb_green_characteristic = false;
static bool trigger_rgb_blue_characteristic = false;

bool flag_temp = true;
bool flag_hum = true;
bool flag_press = true;

bool flag_rgb_red = true;
bool flag_rgb_green = true;
bool flag_rgb_blue = true;

bool connected_to_env = false;
bool connected_to_rgb = false;


void service_discovery_env(const DiscoveredService *service) {
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

void service_discovery_rgb(const DiscoveredService *service) {
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

/*
enum CharacteristicType {
    TEMPERATURE,
    HUMIDITY,
    PRESSURE,
    RED,
    GREEN,
    BLUE
};


void on_characteristic_read(const GattReadCallbackParams *response) {
    CharacteristicType char_type = static_cast<CharacteristicType>(response->handle);

    switch (char_type) {
        case TEMPERATURE: {
            int16_t temperature;
            memcpy(&temperature, response->data, sizeof(temperature));
            printf("Temperature: %.2f\n", temperature / 100.0);
            break;
        }
        case HUMIDITY: {
            uint16_t humidity;
            memcpy(&humidity, response->data, sizeof(humidity));
            printf("Humidity: %.2f\n", humidity / 100.0);
            break;
        }
        case PRESSURE: {
            uint32_t pressure;
            memcpy(&pressure, response->data, sizeof(pressure));
            printf("Pressure: %.2f\n\n", pressure / 10.0);
            break;
        }
        case RED: {
            uint8_t red;
            memcpy(&red, response->data, sizeof(red));
            printf("Red Intensity: %d\n", red);
            break;
        }
        case GREEN: {
            uint8_t green;
            memcpy(&green, response->data, sizeof(green));
            printf("Green Intensity: %d\n", green);
            break;
        }
        case BLUE: {
            uint8_t blue;
            memcpy(&blue, response->data, sizeof(blue));
            printf("Blue Intensity: %d\n\n", blue);
            break;
        }
        default:
            break;
    }
}*/


/* Callback quando la caratteristica viene letta */
void on_characteristic_read(const GattReadCallbackParams *response) {
    
    if (response->handle == temp_characteristic.getValueHandle()) {
        int16_t temperature;
        memcpy(&temperature, response->data, sizeof(temperature));
        printf("Temperature: %.2f\n", temperature / 100.0);
        flag_temp = false;
        flag_hum = true;
    }else if (response->handle == humidity_characteristic.getValueHandle()) {
        uint16_t humidity;
        memcpy(&humidity, response->data, sizeof(humidity));
        printf("Humidity: %.2f\n", humidity / 100.0);
        flag_press = true;
        flag_hum = false;
    }else if (response->handle == pressure_characteristic.getValueHandle()) {
        uint32_t pressure;
        memcpy(&pressure, response->data, sizeof(pressure));
        printf("Pressure: %.2f\n\n", pressure / 10.0);
        flag_temp = true;
    }else if (response->handle == red_characteristic.getValueHandle()) {
        uint8_t red;
        memcpy(&red, response->data, sizeof(red));
        printf("Red Intensity: %d\n", red);
        flag_rgb_green = true;
        flag_rgb_red = false;
    }else if (response->handle == green_characteristic.getValueHandle()) {
        uint8_t green;
        memcpy(&green, response->data, sizeof(green));
        printf("Green Intensity: %d\n", green);
        flag_rgb_blue = true;
        flag_rgb_green = false;
    }else if (response->handle == blue_characteristic.getValueHandle()) {
        uint8_t blue;
        memcpy(&blue, response->data, sizeof(blue));
        printf("Blue Intensity: %d\n\n", blue);
        flag_rgb_red = true;
    }
}

/* funzione che legge tutte le caratteristiche */
void read_all_characteristics() {

    if (!BLE::Instance().gattClient().isServiceDiscoveryActive()) {
        if (trigger_temp_characteristic && (flag_temp == true)) {
            flag_hum = false;
            flag_press = false;
            temp_characteristic.read();
        }

        if (trigger_humidity_characteristic && (flag_hum == true)) {
            flag_press = false;
            humidity_characteristic.read();
        }
        
        if (trigger_pressure_characteristic && (flag_press == true)) {
            pressure_characteristic.read();
        }
/*
        if (trigger_rgb_red_characteristic) {
            red_characteristic.read();
        }

        if (trigger_rgb_green_characteristic) {
            green_characteristic.read();
        }
        
        if (trigger_rgb_blue_characteristic) {
            blue_characteristic.read();
        }*/
    }
}

/* Callback per discovered characteristics */
void characteristic_discovery_env(const DiscoveredCharacteristic *characteristicP) {
    if (characteristicP->getUUID().getShortUUID() == UUID_TEMPERATURE_CHAR) {
        temp_characteristic = *characteristicP;
        trigger_temp_characteristic = true;
    }
    
    if (characteristicP->getUUID().getShortUUID() == UUID_HUMIDITY_CHAR) {
        humidity_characteristic = *characteristicP;
        trigger_humidity_characteristic = true;
    }
    
    if (characteristicP->getUUID().getShortUUID() == UUID_PRESSURE_CHAR) {
        pressure_characteristic = *characteristicP;
        trigger_pressure_characteristic = true;
    }
}

/* Callback per discovered characteristics */
void characteristic_discovery_rgb(const DiscoveredCharacteristic *characteristicP) {
    if (characteristicP->getUUID() == UUID_RED_CHAR) {
        red_characteristic = *characteristicP;
        trigger_rgb_red_characteristic = true;
    }
    
    if (characteristicP->getUUID() == UUID_GREEN_CHAR) {
        green_characteristic = *characteristicP;
        trigger_rgb_green_characteristic = true;
    }
    
    if (characteristicP->getUUID() == UUID_BLUE_CHAR) {
        blue_characteristic = *characteristicP;
        trigger_rgb_blue_characteristic = true;
    }
}

/* Callback per service discovery termination */
void discovery_termination_env(Gap::Handle_t connectionHandle) {
    if (trigger_temp_characteristic || trigger_humidity_characteristic || trigger_pressure_characteristic) {
        event_queue.call(read_all_characteristics);
    }
}

/* Callback per service discovery termination */
void discovery_termination_rgb(Gap::Handle_t connectionHandle) {
    if (trigger_rgb_red_characteristic || trigger_rgb_green_characteristic || trigger_rgb_blue_characteristic) {
        event_queue.call(read_all_characteristics);
    }
}

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
       
        /* Registra la funzione on_characteristic_read come callback per gli eventi di lettura dei dati GATT */
        _ble.gattClient().onDataRead(on_characteristic_read);

        /* Definisce e imposta i parametri di scansione BLE. In questo caso, vengono utilizzati i parametri di default. */ 
        ble::ScanParameters scan_params;
        _ble.gap().setScanParameters(scan_params);
        _ble.gap().startScan();
    }

    void update_sensor_values() {
        if (trigger_temp_characteristic || trigger_humidity_characteristic || trigger_pressure_characteristic ||
            trigger_rgb_red_characteristic || trigger_rgb_green_characteristic || trigger_rgb_blue_characteristic) {
            read_all_characteristics();
        }
    }

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startScan();
        _is_connecting = false;
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent& event) {
        if (event.getOwnRole() == ble::connection_role_t::CENTRAL) {
            
            if (event.getPeerAddress() == ENV_MAC_ADDRESS) {
                _ble.gattClient().onServiceDiscoveryTermination(discovery_termination_env);
                _ble.gattClient().launchServiceDiscovery(
                    event.getConnectionHandle(),
                    service_discovery_env,
                    characteristic_discovery_env,
                    UUID_ENVIRONMENTAL_SERVICE
                );
                printf("Primo if in onConnectionComplete");
            }

            if (event.getPeerAddress() == RGB_MAC_ADDRESS) {
                _ble.gattClient().onServiceDiscoveryTermination(discovery_termination_rgb);
                _ble.gattClient().launchServiceDiscovery(
                    event.getConnectionHandle(),
                    service_discovery_rgb,
                    characteristic_discovery_rgb,
                    UUID_RGB_SERVICE
                );
            }
        }
        
        _is_connecting = false;

        if (!connected_to_env || !connected_to_rgb) {
            _ble.gap().startScan();
        }
    }

     void onAdvertisingReport(const ble::AdvertisingReportEvent &event) {

        ble::AdvertisingDataParser adv_data(event.getPayload());

        /* parse the advertising payload, looking for a discoverable device */
        while (adv_data.hasNext()) {
            ble::AdvertisingDataParser::element_t field = adv_data.next();

            /* connect to a discoverable device */
            if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME &&
                ((field.value.size() == strlen(PEER_NAME) &&
                memcmp(field.value.data(), PEER_NAME, field.value.size()) == 0) ||
                (field.value.size() == strlen(PEER2_NAME) &&
                memcmp(field.value.data(), PEER2_NAME, field.value.size()) == 0))) {

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

                if ((memcmp(field.value.data(), PEER_NAME, field.value.size()) == 0)) {
                    connected_to_env = true;
                    printf("CONNESSO a %s\n", PEER_NAME);
                }
                
                if (memcmp(field.value.data(), PEER2_NAME, field.value.size()) == 0) {
                    connected_to_rgb = true;
                    printf("CONNESSO a %s\n", PEER2_NAME);
                }

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
    pc.printf("Inizio\n");

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    Client myclient(ble, event_queue);

    myclient.start();

    return 0;
}
