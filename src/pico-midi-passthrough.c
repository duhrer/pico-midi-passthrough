// Adapted from https://github.com/infovore/pico-example-midi

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

// Set the "data plus" pin for PIO USB to 16, which is what the Adafruit feather rp2040 with USB Host uses.
//#define PIO_USB_DP_PIN      16

// Set the "data plus" pin for PIO USB to 12 for the Waveshare 2350 board
#define PIO_USB_DP_PIN      12


#define PIO_USB_CONFIG { \
    PIO_USB_DP_PIN, \
    PIO_USB_TX_DEFAULT, \
    PIO_SM_USB_TX_DEFAULT, \
    PIO_USB_DMA_TX_DEFAULT, \
    PIO_USB_RX_DEFAULT, \
    PIO_SM_USB_RX_DEFAULT, \
    PIO_SM_USB_EOP_DEFAULT, \
    NULL, \
    PIO_USB_DEBUG_PIN_NONE, \
    PIO_USB_DEBUG_PIN_NONE, \
    false, \
    PIO_USB_PINOUT_DPDM \
}

#include "pio_usb.h"
#include "tusb.h"

void midi_client_task(void);
void midi_host_task(void);

void core1_main() {
  sleep_ms(10);

  pio_usb_configuration_t pio_cfg = PIO_USB_CONFIG;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

  tuh_init(1);

  while (true) {
    tuh_task();
  }
}

int main() {
  // Enable USB power for client devices, nicked from OGX MINI:
  // https://github.com/wiredopposite/OGX-Mini/blob/ea14d683adeea579228109d2f92a4465fb76974d/Firmware/RP2040/src/Board/board_api_private/board_api_usbh.cpp#L35

  // gpio_init(18);
  // gpio_set_dir(18, GPIO_OUT);
  // gpio_put(18, 1);

  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(120000, true);

  sleep_ms(10);

  multicore_reset_core1();
  multicore_launch_core1(core1_main);

  // device stack on native USB
  tud_init(0);

  while (true)
  {
    tud_task(); // tinyusb device task

    midi_client_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Client Callbacks

// Invoked when device is mounted
void tud_mount_cb(void) {}

// Invoked when device is unmounted
void tud_umount_cb(void) {}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {}

// Host Callbacks

// The empty placeholder callbacks would ordinarily throw warnings about unused variables, so we use the strategy outlined here:
// https://stackoverflow.com/questions/3599160/how-can-i-suppress-unused-parameter-warnings-in-c

// Invoked when device with MIDI interface is mounted.
static uint8_t client_device_idx = 0;

void tuh_midi_mount_cb(uint8_t idx, __attribute__((unused)) const tuh_midi_mount_cb_t* mount_cb_data) {
  client_device_idx = idx;
}

// Invoked when device with hid interface is un-mounted
void tuh_midi_umount_cb(__attribute__((unused)) uint8_t idx) {
  client_device_idx = 0;
}

// Host -> Client Relay
void tuh_midi_rx_cb(uint8_t idx, uint32_t xferred_bytes) {
  if (xferred_bytes == 0) {
    return;
  }

  // Simplistic packet-based approach used in a a few demos, stable, probably not adequate for sysex.
  //
  // uint8_t incoming_packet[4];
  // while (tuh_midi_packet_read(idx, incoming_packet)) {
  //   uint8_t midi_data[3];
  //   memcpy(midi_data, incoming_packet + 1, 3);

  //   // static inline uint32_t tud_midi_stream_write (uint8_t cable_num, uint8_t const* buffer, uint32_t bufsize);
  //   tud_midi_stream_write(0, midi_data, 3);
  // }

  // Stream-based approach, intermittently flaky.
  uint8_t *buffer;
  buffer = (uint8_t*) malloc(xferred_bytes);

  uint8_t cable_num = 0;

  // uint32_t tuh_midi_stream_read(uint8_t idx, uint8_t *p_cable_num, uint8_t *p_buffer, uint16_t bufsize);
  tuh_midi_stream_read(idx, &cable_num, buffer, sizeof(buffer));

  // static inline uint32_t tud_midi_stream_write (uint8_t cable_num, uint8_t const* buffer, uint32_t bufsize);
  tud_midi_stream_write(cable_num, buffer, sizeof(buffer));

  free(buffer);
}

void tuh_midi_tx_cb(uint8_t idx, uint32_t xferred_bytes) {
  (void) idx;
  (void) xferred_bytes;
}

// End TinyUSB Callbacks

//--------------------------------------------------------------------+
// MIDI Tasks
//--------------------------------------------------------------------+

// Client -> Host Relay
void midi_client_task(void)
{
  // If a MIDI message falls in the forest and there's no one there to receive
  // it, it definitely doesn't get sent.
  if (!tuh_midi_mounted(client_device_idx)) { return; }

  uint32_t bytes_written = 0;

  // Simplistic packet-based approach used in a a few demos, stable, probably not adequate for sysex.
  //
  // // Read any incoming messages from our primary USB port.
  // while (tud_midi_available()) {
  //   uint8_t incoming_packet[4];
  //   tud_midi_packet_read(incoming_packet);

  //   uint8_t midi_data[3];
  //   memcpy(midi_data, incoming_packet + 1, 3);

  //   // TODO: This is probably too specific to the Launchpad, need to enumerate "cables" instead.
  //   bytes_written += tuh_midi_stream_write(client_device_idx, 1, midi_data, 3);

  //   // TODO: See if we can pass through raw packets instead.
  //   // uint32_t tuh_midi_packet_write_n(uint8_t idx, const uint8_t* buffer, uint32_t bufsize);
  //   // tuh_midi_packet_write_n(1, incoming_packet, sizeof(incoming_packet));
  // }

  // Stream-based approach, seems more solid on this side.
  uint32_t bytes_to_read = tud_midi_available();

  if (bytes_to_read) {
    uint8_t *buffer;
    buffer = (uint8_t*) malloc(bytes_to_read);

    // static inline uint32_t tud_midi_stream_read (void* buffer, uint32_t bufsize)
    tud_midi_stream_read(buffer, sizeof(buffer));

    for (uint8_t cable_num = 0; cable_num < tuh_midi_get_tx_cable_count(client_device_idx); cable_num++) {
      // uint32_t tuh_midi_stream_write(uint8_t idx, uint8_t cable_num, uint8_t const *p_buffer, uint32_t bufsize);
      bytes_written += tuh_midi_stream_write(client_device_idx, cable_num, buffer, sizeof(buffer));     
    }

    if (bytes_written > 0) {
      tuh_midi_write_flush(client_device_idx);
    }

    // We can't free the buffer until we've finished passing the data and flushed the write cache.
    free(buffer);
  }
}

void midi_host_task(void) {
  // TODO: Do something?
}