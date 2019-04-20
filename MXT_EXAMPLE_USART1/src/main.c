/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch low level component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and Xmega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sent over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incoming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * Baud: 57600
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

///////////////////////////////////////////////////////////////////////////////////// INCLUDES
#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "font_24.h"
#include "font_invert_24.h"

// Icons
#include "icones/play.h"
#include "icones/lock.h"
#include "icones/pause.h"
#include "icones/corsi.h"

// Mode buttons
#include "icones/fast_mode.h"
#include "icones/heavy_mode.h"
#include "icones/rinse_mode.h"
#include "icones/daily_mode.h"

#include "maquina1.h"

///////////////////////////////////////////////////////////////////////////////////// DEFINES
#define MAX_ENTRIES        3
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

//Led placa
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

//Botão placa
#define BUT_PIO_ID	   ID_PIOA
#define BUT_PIO        PIOA
#define BUT_PIN		   11
#define BUT_IDX_MASK   (1<<BUT_PIN)

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 32;
const uint32_t BUTTON_H = 60;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH-35/2;
const uint32_t BUTTON_Y = 60/2;

const uint32_t TIMER_Y = ILI9488_LCD_HEIGHT- 30;

const uint32_t CENTER_X = ILI9488_LCD_WIDTH/2;
const uint32_t CENTER_Y = ILI9488_LCD_HEIGHT/2;

const uint32_t QUICK_PLAY_W = 300;
const uint32_t QUICK_PLAY_H = 100 ;
const uint32_t QUICK_PLAY_X = ILI9488_LCD_WIDTH - 160;
const uint32_t QUICK_PLAY_Y = ILI9488_LCD_HEIGHT - 60 ;

int triggered = 0;
int next = 0;
int previous = 0;
int locked = 0;
int status_screen = 0;
int margin = 50;

volatile Bool porta_aberta = 0;
volatile Bool lavando = 0;
volatile Bool reload_screen = 1;
volatile char nome[32];
volatile char tempo[32];
volatile char centrifugacao[32];
volatile char tempo_lavagem[32];
volatile t_ciclo *ciclo_atual;
volatile int counter_seg = 0;
volatile int counter_min = 0;
volatile int counter_hor = 0;
volatile int duracao;

///////////////////////////////////////////////////////////////////////////////////// Draw

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

/**
 * \brief Draw a pixmap on LCD.
 *
 * \param ul_x X coordinate of upper-left corner on LCD.
 * \param ul_y Y coordinate of upper-left corner on LCD.
 * \param ul_width width of the picture.
 * \param ul_height height of the picture.
 * \param p_ul_pixmap pixmap of the image.
 */
void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	ili9488_draw_pixmap(0,0, corsi.width, corsi.height, corsi.data);
}

///////////////////////////////////////////////////////////////////////////////////// CALLBACKS E HANDLERS
void but_callback(void)
{
	if(!lavando){
		if(porta_aberta){
			pio_set(LED_PIO, LED_IDX_MASK);
			porta_aberta = 0;
			}else{
			pio_clear(LED_PIO, LED_IDX_MASK);
			porta_aberta = 1;
		}
	}
}

void RTC_Handler(void){
	
	int hora_atual = 0;
	int min_atual = 0;
	int seg_atual = 0;
	counter_hor = 0;
	counter_min = 0;
	counter_seg = 0;
	lavando = 0;
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_set_date_alarm(RTC, 1, 0, 1, 0);
	rtc_get_time(RTC, &hora_atual, &min_atual, &seg_atual);
	
	counter_seg += 1;
	if (seg_atual>=59){
		counter_min += 1;
		counter_seg = 0;
		if(min_atual>=59){
			counter_hor += 1;
			counter_min = 0;
			rtc_set_time_alarm(RTC, 1, hora_atual+1, 1, 0, 1, 0);
		}
		else{
			counter_min +=1;
			rtc_set_time_alarm(RTC, 1, hora_atual, 1, min_atual+1, 1, 0);
		}
	}
	else{
		rtc_set_time_alarm(RTC, 1, hora_atual, 1, min_atual, 1, seg_atual+1);
	}
	
	if(duracao - counter_seg >= 0){
		duracao -= 1;
		sprintf(tempo_lavagem, "%02d:%02d:%02d", duracao/3600, duracao%3600/60, duracao%3600%60);
		font_draw_text(&font_24, tempo_lavagem, 10, TIMER_Y-10, 1);
		
		}else{
		reload_screen = 1;
	}
	if (duracao == 0){
		reload_screen = 1;
		lavando = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////// RTC

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, 0, 0, 0, 0);
	rtc_set_time(RTC, 0, 0, 0);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);
}

///////////////////////////////////////////////////////////////////////////////////// CONFIGURES E LED
/**
 * Inicializa ordem do menu
 * retorna o primeiro ciclo que
 * deve ser exibido.
 */
t_ciclo *initMenuOrder(){
  c_rapido.previous = &c_enxague;
  c_rapido.next = &c_diario;

  c_diario.previous = &c_rapido;
  c_diario.next = &c_pesado;

  c_pesado.previous = &c_diario;
  c_pesado.next = &c_enxague;

  c_enxague.previous = &c_pesado;
  c_enxague.next = &c_centrifuga;

  c_centrifuga.previous = &c_enxague;
  c_centrifuga.next = &c_rapido;

  return(&c_diario);
}

static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);
	
	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

/**
 * \brief Set maXTouch configuration
 *
 * This function writes a set of predefined, optimal maXTouch configuration data
 * to the maXTouch Xplained Pro.
 *
 * \param device Pointer to mxt_device struct
 */

static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void draw_lock_button(uint32_t clicked) {
	static uint32_t last_state = 255; // undefined
	if(clicked == last_state) return;
	
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE/3));
	ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2, BUTTON_Y-BUTTON_H/2, BUTTON_X+BUTTON_W/2, BUTTON_Y+BUTTON_H/2);
	if(clicked) {
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_ORANGE));
		ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y+BUTTON_H/2-BUTTON_BORDER);
		ili9488_draw_pixmap(BUTTON_X-15, BUTTON_Y, lock.width, lock.height, lock.data);
	} else {
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_RED));
		ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y-BUTTON_H/2+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y-BUTTON_BORDER);
		ili9488_draw_pixmap(BUTTON_X-15, BUTTON_Y-30, lock.width, lock.height, lock.data);
	}
	last_state = clicked;
}

void draw_quick_play_button(uint32_t clicked) {
	duracao = (ciclo_atual->enxagueTempo*ciclo_atual->enxagueQnt) + ciclo_atual->centrifugacaoTempo;
	static uint32_t last_state = 255; // undefined
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE/3));
	ili9488_draw_filled_rectangle(QUICK_PLAY_X, QUICK_PLAY_Y, QUICK_PLAY_X+QUICK_PLAY_W/2, QUICK_PLAY_Y+QUICK_PLAY_H/2);
	if(clicked){
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
		ili9488_draw_filled_rectangle(QUICK_PLAY_X+BUTTON_BORDER, QUICK_PLAY_Y+BUTTON_BORDER, QUICK_PLAY_X+QUICK_PLAY_W/2-BUTTON_BORDER, QUICK_PLAY_Y+QUICK_PLAY_H/2-BUTTON_BORDER);
		font_draw_text(&font_invert_24, "Cancel",QUICK_PLAY_X+10, QUICK_PLAY_Y+10, 1);
		ili9488_draw_pixmap(QUICK_PLAY_X+110, QUICK_PLAY_Y+10, pause.width, pause.height, pause.data);
	} else{
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
		ili9488_draw_filled_rectangle(QUICK_PLAY_X+BUTTON_BORDER, QUICK_PLAY_Y+BUTTON_BORDER, QUICK_PLAY_X+QUICK_PLAY_W/2-BUTTON_BORDER, QUICK_PLAY_Y+QUICK_PLAY_H/2-BUTTON_BORDER);
		font_draw_text(&font_24, "Play",QUICK_PLAY_X+10, QUICK_PLAY_Y+10, 1);
		ili9488_draw_pixmap(QUICK_PLAY_X+110, QUICK_PLAY_Y+10, play.width, play.height, play.data);
	}
	
}

void draw_next_button() {
		static uint32_t last_state = 255;
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GRAY));
		ili9488_draw_filled_rectangle(ILI9488_LCD_WIDTH/2+10, 328, ILI9488_LCD_WIDTH/2+80, 368);
}

void draw_previous_button() {
	static uint32_t last_state = 255;
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GRAY));
	ili9488_draw_filled_rectangle(ILI9488_LCD_WIDTH/2-80, 328, ILI9488_LCD_WIDTH/2-10, 368);
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

play_button(uint32_t tx, uint32_t ty){
	if(tx >= QUICK_PLAY_X && tx <= QUICK_PLAY_X + QUICK_PLAY_W/2) {
		if(ty >= QUICK_PLAY_Y && ty <= QUICK_PLAY_Y + QUICK_PLAY_H) {
			if(!porta_aberta){
				triggered = ~triggered;
				if(triggered){
					lavando = 1;
					reload_screen = 1;
				}
				else {
					lavando = 0;
					reload_screen = 1;
				}	
			}else{
				lavando = 0;
				reload_screen = 1;
			}
		}
	}
}

next_button(uint32_t tx, uint32_t ty){
		if(tx >= ILI9488_LCD_WIDTH/2-40+15 && tx <= ILI9488_LCD_WIDTH/2+40+15) {
			if(ty >= 328+15 && ty <= 368+15) {
				next = ~next;
				if(next){
					ciclo_atual = ciclo_atual->next;
					reload_screen = 1;
				}
			}
		}
}

previous_button(uint32_t tx, uint32_t ty){
		if(tx >= ILI9488_LCD_WIDTH/2-80+15 && tx <= ILI9488_LCD_WIDTH/2-10) {
			if(ty >= 328+15 && ty <= 368+15) {
				previous = ~previous;
				if(previous){
					ciclo_atual = ciclo_atual->previous;
					reload_screen = 1;
				}
			}
		}
}

draw_mode_button(){
	ili9488_draw_pixmap(30,CENTER_Y-(fast.height+5)*3+margin, heavy.width, heavy.height, heavy.data);
	ili9488_draw_pixmap(30,CENTER_Y-(fast.height+5)*2+margin, daily.width, daily.height, daily.data);
	ili9488_draw_pixmap(30,CENTER_Y-(fast.height+5)+margin, rinse.width, rinse.height, rinse.data);
	ili9488_draw_pixmap(30,CENTER_Y+margin, fast.width, fast.height, fast.data);
}

void update_screen(uint32_t tx, uint32_t ty) {
	//font_draw_text(Font *font, const char* texto, int x, int y, int spacing)
	// Lock Button
	if(tx >= BUTTON_X-BUTTON_W/2 && tx <= BUTTON_X + BUTTON_W/2) {
		if(ty >= BUTTON_Y-BUTTON_H/2 && ty <= BUTTON_Y) {
			draw_lock_button(1);
			locked = 1;
		} else if(ty > BUTTON_Y && ty < BUTTON_Y + BUTTON_H/2) {
			draw_lock_button(0);
			locked = 0;
		}
	}
	if (locked == 0 ){
		play_button(tx,ty);
		next_button(tx,ty);
		previous_button(tx,ty);
		//draw_mode_button();
		}
}

void mxt_handler(struct mxt_device *device)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
	
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		 // eixos trocados (quando na vertical LCD)
		uint32_t conv_x = convert_axis_system_x(touch_event.y);
		uint32_t conv_y = convert_axis_system_y(touch_event.x);
		
		/* Format a new entry in the data string that will be sent over USART */
		sprintf(buf, "Nr: %1d, X:%4d, Y:%4d, Status:0x%2x conv X:%3d Y:%3d\n\r",
				touch_event.id, touch_event.x, touch_event.y,
				touch_event.status, conv_x, conv_y);
		
		if (status_screen <= 42){
			update_screen(conv_x, conv_y);
		}
		status_screen = touch_event.status;
		

		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, strlen(tx_buf));
	}
}

void init(void){
	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	configure_lcd();
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
	
	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	
	//callback
	pio_handler_set(BUT_PIO,BUT_PIO_ID,BUT_IDX_MASK,PIO_IT_RISE_EDGE,but_callback);
	
	// Ativa interrupção
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);

	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 0); 
}

int main(void)
{
	ciclo_atual = initMenuOrder();
	init();
	RTC_init();
	/* Initialize the mXT touch device */
	struct mxt_device device;
	mxt_init(&device);
	
	//apaga o led pois a porta sempre começa fechada
	pio_set(LED_PIO, LED_IDX_MASK);
	porta_aberta = 0;
	
	while (1) {
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device);
		}
		if(reload_screen){
			draw_screen();
			draw_lock_button(0);
			draw_next_button();
			draw_previous_button();
			draw_quick_play_button(triggered);
			
			sprintf(nome,"Ciclo: %s", ciclo_atual->nome);
			font_draw_text(&font_24, nome, 5, 100, 1);
			sprintf(centrifugacao,"Centrifugacao: %d", ciclo_atual->centrifugacaoRPM);
			font_draw_text(&font_24, centrifugacao, 5, 150, 1);
			if(ciclo_atual->bubblesOn){
				font_draw_text(&font_24, "Bolhas: Ligadas", 5, 200, 1);	
			}else{
				font_draw_text(&font_24, "Bolhas: Desligadas", 5, 200, 1);
			}
			
			duracao = (ciclo_atual->centrifugacaoTempo + (ciclo_atual->enxagueTempo*ciclo_atual->enxagueQnt)) * 60;
			sprintf(tempo_lavagem, "%02d:%02d:%02d", duracao/3600, duracao%3600/60, duracao%3600%60);
			font_draw_text(&font_24, tempo_lavagem, 10, TIMER_Y-10, 1);
			
			reload_screen = 0;
		}
		if(lavando){
			rtc_set_date_alarm(RTC, 1, 0, 1, 0);
			rtc_set_time_alarm(RTC, 0, 0, 1, 0, 1, 1);
		}
	}
	return 0;
}
