// Raspberry Pi Pico as a GPIO to I2C multiplexer
// Uses pins 0, 1 for I2C
// Reads/writes the pin states from 2 to 21

#include <stdio.h>
#include "pico/stdlib.h"
#include <i2c_fifo.h>
#include <i2c_slave.h>

#ifndef USB_DEBUG
#define USB_DEBUG 1
#endif

#define I2C_PORT i2c0
#define I2C_SLAVE_SDA_PIN 0
#define I2C_SLAVE_SCL_PIN 1
#define I2C_SLAVE_ADDRESS 0x20
#define I2C_BAUDRATE 100000 // 100kHz

#define MEM_SIZE 256

// I2C registers
// 0x00 - 0x13: Pin states (0 = input, 1 = output)
// 0x14 - 0x27: Pin values (0 = low, 1 = high)

static struct
{
	uint8_t mem[MEM_SIZE];
    uint8_t mem_address;
    bool mem_address_written;
} context;

// Pin states: 0 = input, 1 = output
int pin_states[20];
// Pin values: 0 = low, 1 = high
int pin_values[20];

int pin = 2;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE:
        if (!context.mem_address_written) {
            context.mem_address = i2c_read_byte(i2c);
            context.mem_address_written = true;
        } else {
            context.mem[context.mem_address] = i2c_read_byte(i2c);

			// Handle pin states
			if (context.mem_address < 0x14) {
				pin = context.mem_address + 2;
				int state = context.mem[context.mem_address];
				if (pin_states[pin] != state) {
					pin_states[pin] = state;
					if (state == 0) {
						gpio_set_dir(pin, GPIO_IN);
					} else {
						gpio_set_dir(pin, GPIO_OUT);
					}
				}
			}
			
			// Handle pin values
			if (context.mem_address >= 0x14) {
				pin = context.mem_address - 0x14 + 2;
				int value = context.mem[context.mem_address];	
				if (pin_states[pin] == 1) {
					if (pin_values[pin] != value) {
						pin_values[pin] = value;
						gpio_put(pin, value);
					}
				}
			}

            context.mem_address++;

			// Stop at the end of the memory
			if (context.mem_address >= MEM_SIZE) {
				context.mem_address = MEM_SIZE - 1;
			}
        }
        break;
    case I2C_SLAVE_REQUEST:
		// Handle pin values
		pin = context.mem_address - 0x14 + 2;
		if (pin >= 2 && pin <= 21) {
			if (pin_states[pin] == 0) {
				pin_values[pin] = gpio_get(pin);
			}
			context.mem[context.mem_address] = pin_values[pin];
		}
        i2c_write_byte(i2c, context.mem[context.mem_address]);
        context.mem_address++;
        break;
    case I2C_SLAVE_FINISH:
        context.mem_address_written = false;
        break;
    default:
        break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c1, I2C_BAUDRATE);
    i2c_slave_init(i2c1, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

int main() {
	stdio_init_all();

	// Initialize pin states
	for (int i = 2; i <= 21; i++) {
		pin_states[i - 2] = 0;
		pin_values[i - 2] = 0;
		gpio_init(i);
		gpio_set_dir(i, GPIO_IN);
	}

	setup_slave();

	while (true) {
		tight_loop_contents();
		#if USB_DEBUG
		printf("[");
		for (int i = 2; i <= 21; i++) {
			printf("%d ", gpio_get(i));
		}
		printf("]\n");
		sleep_ms(1000);
		#endif
	}

	return 0;
}