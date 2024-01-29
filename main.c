#define F_CPU 4000000
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>
#include <util/delay_basic.h>
#include <stdio.h>
#include "bme680.h"
#include "bme680_defs.h"

uint8_t status1;
uint8_t status2;
uint8_t id;
uint8_t temp_msb;
uint8_t temp_lsb;
uint8_t temp_xlsb;
uint32_t temp_raw;
char dsp_buff1[17] = "temp: ";
char dsp_buff2[17] = "hum: ";
char dsp_buff3[17] = "press: ";

void TWI0_init() {
    TWI0_MSTATUS |= 0x01;
    TWI0_MSTATUS &= ~0x02; //setting to idle state
    TWI0_MCTRLA = (TWI_RIEN_bm | TWI_WIEN_bm | TWI_ENABLE_bm);
    TWI0_MBAUD = 1; //303 kHz it is
}

uint8_t BME680_TWI_read(uint8_t reg_addr) {
    uint8_t data;
    TWI0_MADDR = 0b11101100; //BME's slave address (0 for write)
    while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    TWI0_MDATA = reg_addr;
    while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from 
    BME
    TWI0_MADDR = 0b11101101; //BME's slave address (1 for read)
    // while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    while(!(TWI0_MSTATUS & TWI_RIF_bm)) { ; } //waiting for reading to complete
    data = TWI0_MDATA;
    TWI0_MCTRLB = (TWI_ACKACT_bm | TWI_MCMD1_bm | TWI_MCMD0_bm); //sends NACK and stop bit
    return data;
}

void BME680_TWI_write(uint8_t reg_addr, uint8_t data) {
    TWI0_MADDR = 0b11101100; //BME's slave address (0 for write)
    while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    TWI0_MDATA = reg_addr;
    while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    TWI0_MDATA = data;
    while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    TWI0_MCTRLB = (TWI_ACKACT_bm | TWI_MCMD1_bm | TWI_MCMD0_bm); //sends NACK and stop bit
}

void TWI_BME680_init() {
    BME680_TWI_write(0xE0, 0xB6);
    _delay_us(100);
    status1 = BME680_TWI_read(0x73);
    _delay_us(100);
    id = BME680_TWI_read(0xD0);
    _delay_us(100);
    status2 = BME680_TWI_read(0x73);
    _delay_us(100);
}

uint8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    uint8_t rslt = 1;
    dev_id &= ~0x01; //0 for write
    TWI0_MADDR = dev_id; 
    while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    for(uint8_t i=0; i<len; i++) {
        TWI0_MDATA = reg_addr;
        while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to 
        complete
        while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge 
        from BME
        TWI0_MDATA = *(reg_data+i);
        while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to 
        complete
        while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge 
        from BME
        reg_addr++;
    }

    TWI0_MCTRLB = (TWI_ACKACT_bm | TWI_MCMD1_bm | TWI_MCMD0_bm); //sends NACK and stop bit
    return rslt;
}

uint8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    uint8_t rslt = 0;
    TWI0_MADDR = 0b11101100; //BME's slave address (0 for write)
    while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    TWI0_MDATA = reg_addr;
    while(!(TWI0_MSTATUS & TWI_WIF_bm)) { ; } //waiting for write to complete
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    TWI0_MADDR = 0b11101101; //BME's slave address (1 for read)
    for(int i=0; i<(len-1); i++) {
        while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
        while(!(TWI0_MSTATUS & TWI_RIF_bm)) { ; } //waiting for reading to complete
        *(reg_data+i) = TWI0_MDATA;
        TWI0_MCTRLB = 0b00000010; //send ACKM
    }
    while(TWI0_MSTATUS & TWI_RXACK_bm) { ; } //waiting for acknowledge from BME
    while(!(TWI0_MSTATUS & TWI_RIF_bm)) { ; } //waiting for reading to complete
    *(reg_data+len-1) = TWI0_MDATA;
    TWI0_MCTRLB = (TWI_ACKACT_bm | TWI_MCMD1_bm | TWI_MCMD0_bm); //sends NACK and stop bit
    return rslt;
}

void user_delay_ms(uint32_t period) {
    _delay_ms(period);
}

void lcd_spi_init() {
    PORTA.DIR |= PIN4_bm; /* Set MOSI pin direction to output */
    PORTA.DIR |= PIN6_bm; /* Set SCK pin direction to output */
    PORTA.DIR |= PIN7_bm; /* Set SS pin direction to output */
    PORTC.DIR |= PIN0_bm; // Set RS pin direction to output
    SPI0.CTRLA = SPI_ENABLE_bm /* Enable module */
    | SPI_MASTER_bm; /* SPI module in Host mode */
    SPI0.CTRLB |= (SPI_MODE_3_gc); //Leading edge is falling, sampling done onrising edge
}

void lcd_spi_transmit_CMD (unsigned char cmd) {
    PORTC_OUT &= ~PIN0_bm; // RS = 0 for command
    // SPI0_CTRLB &= ~SPI_SSD_bm; //assert slave select, not needed when MSSEN=1
    SPI0_DATA = cmd; //send command
    while (!(SPI0.INTFLAGS & SPI_IF_bm)) { ; } // wait until Tx ready
    // SPI0_CTRLB |= SPI_SSD_bm; //unassert slave select, not needed when MSSEN= 1
}

void lcd_spi_transmit_DATA (unsigned char cmd) {
    PORTC_OUT |= PIN0_bm; // RS = 1 for command
    // SPI0_CTRLB &= ~SPI_SSD_bm; //assert slave select, not needed when MSSEN=1
    SPI0_DATA = cmd; //send command
    while (!(SPI0.INTFLAGS & SPI_IF_bm)) { ; } // wait until Tx ready
    // SPI0_CTRLB |= SPI_SSD_bm; //unassert slave select, not needed when MSSEN= 1
}

void init_lcd_dog (void) {
    _delay_ms(40); //startup delay.
    //func_set1:
    lcd_spi_transmit_CMD(0x39); // send function set #1
    _delay_us(30); //delay for command to be processed
    //func_set2:
    lcd_spi_transmit_CMD(0x39); //send function set #2
    _delay_us(30); //delay for command to be processed
    //bias_set:
    lcd_spi_transmit_CMD(0x1E); //set bias value.
    _delay_us(30); //delay for command to be processed
    //power_ctrl:
    lcd_spi_transmit_CMD(0x55); //~ 0x50 nominal for 5V
    //~ 0x55 for 3.3V (delicate adjustment).
    _delay_us(30); //delay for command to be processed
    //follower_ctrl:
    lcd_spi_transmit_CMD(0x6C); //follower mode on...
    _delay_ms(40); //delay for command to be processed
    //contrast_set:
    lcd_spi_transmit_CMD(0x7F); //~ 77 for 5V, ~ 7F for 3.3V
    _delay_us(30); //delay for command to be processed
    //display_on:
    lcd_spi_transmit_CMD(0x0c); //display on, cursor off, blink off
    _delay_us(30); //delay for command to be processed
    //clr_display:
    lcd_spi_transmit_CMD(0x01); //clear display, cursor home
    _delay_us(30); //delay for command to be processed
    //entry_mode:
    lcd_spi_transmit_CMD(0x06); //clear display, cursor home
    _delay_us(30); //delay for command to be processed
}

void update_lcd_dog(void) {
    // send line 1 to the LCD module.
    lcd_spi_transmit_CMD(0x80); //init DDRAM addr-ctr
    _delay_us(30);
    for (int i = 0; i < 16; i++) {
        lcd_spi_transmit_DATA(dsp_buff1[i]);
        _delay_us(30);
    }
    // send line 2 to the LCD module.
    lcd_spi_transmit_CMD(0x90); //init DDRAM addr-ctr
    _delay_us(30);
    for (int i = 0; i < 16; i++) {
        lcd_spi_transmit_DATA(dsp_buff2[i]);
        _delay_us(30);
    }
    // send line 3 to the LCD module.
    lcd_spi_transmit_CMD(0xA0); //init DDRAM addr-ctr
    _delay_us(30);
    for (int i = 0; i < 16; i++) {
        lcd_spi_transmit_DATA(dsp_buff3[i]);
        _delay_us(30);
    }
}

int main(void)
{
    struct bme680_dev gas_sensor;
    gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
    gas_sensor.intf = BME680_I2C_INTF;
    gas_sensor.read = user_i2c_read;
    gas_sensor.write = user_i2c_write;
    gas_sensor.delay_ms = user_delay_ms;
    gas_sensor.amb_temp = 25;
    int8_t rslt = BME680_OK;
    rslt = bme680_init(&gas_sensor);
    uint8_t set_required_settings;
    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */
    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE;
    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL |
    BME680_FILTER_SEL
    | BME680_GAS_SENSOR_SEL;
    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);
    TWI0_init();
    TWI_BME680_init();
    BME680_TWI_write(0x74, 0x21);
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &gas_sensor);
    struct bme680_field_data data ;
    while(1) {
        user_delay_ms(meas_period); /* Delay till the measurement is ready */
        rslt = bme680_get_sensor_data(&data, &gas_sensor);
        sprintf(dsp_buff1, "Temp: %.2f", data.temperature / 100.0f);
        sprintf(dsp_buff2, "Pres: %.2f", data.pressure / 100.0f);
        sprintf(dsp_buff3, "Hum: %.2f", data.humidity / 1000.0f);
        printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH " , data.temperature / 100.0f,
        data.pressure / 100.0f, data.humidity / 1000.0f );
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            printf(", G: %ld ohms", data.gas_resistance);

        printf("\r\n");
        /* Trigger the next measurement if you would like to read data out 
        continuously */
        if (gas_sensor.power_mode == BME680_FORCED_MODE) {
            rslt = bme680_set_sensor_mode(&gas_sensor);
            update_lcd_dog();
        }
    }
}