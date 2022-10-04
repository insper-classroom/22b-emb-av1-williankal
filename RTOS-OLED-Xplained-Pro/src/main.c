#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)	

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOC
#define BUT_3_PIO_ID ID_PIOC
#define BUT_3_IDX 31
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

#define IN_1_PIO PIOD
#define IN_1_PIO_ID ID_PIOD
#define IN_1_IDX 30
#define IN_1_IDX_MASK (1u << IN_1_IDX)

#define IN_2_PIO PIOA
#define IN_2_PIO_ID ID_PIOA
#define IN_2_IDX 6
#define IN_2_IDX_MASK (1u << IN_2_IDX)

#define IN_3_PIO PIOC
#define IN_3_PIO_ID ID_PIOC
#define IN_3_IDX 19
#define IN_3_IDX_MASK (1u << IN_4_IDX)

#define IN_4_PIO PIOA
#define IN_4_PIO_ID ID_PIOA
#define IN_4_IDX 2
#define IN_4_IDX_MASK (1u << IN_4_IDX)


/** RTOS  */
#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_MOTOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY            (tskIDLE_PRIORITY)


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void io_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);


QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;
SemaphoreHandle_t xSemaphoreRTT;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

void RTT_HANDLER(void) {
  uint32_t ul_status;
  ul_status = rtt_get_status(RTT);

  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
	xSemaphoreGiveFromISR(xSemaphoreRTT, 0);
   }  
}
/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_oled1_callback(void){
	int16_t inc = 180;
	xQueueSendFromISR(xQueueModo, &inc, 0);
}

void but_oled2_callback(void){
	int16_t inc = 90;
	xQueueSendFromISR(xQueueModo, &inc, 0);
	}

void but_oled3_callback(void){
	int16_t inc = 45;
	xQueueSendFromISR(xQueueModo, &inc, 0);
	}

void apaga_tela() {
	gfx_mono_draw_filled_rect(0, 0, 120, 30, GFX_PIXEL_CLR);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) { 
 	int16_t inc = 0;
  for (;;) {
    if (xQueueReceive(xQueueModo, &inc, 10)) {
      int graus = inc;
	  int passos = (graus / 0,17578125);

		if(graus == 180){
			apaga_tela();
			gfx_mono_draw_string("180", 50, 12, &sysfont);
			xQueueSendFromISR(xQueueSteps, &passos, 0);}
		if(graus == 45){
			apaga_tela();
			gfx_mono_draw_string("45", 50, 12, &sysfont);
			xQueueSendFromISR(xQueueSteps, &passos, 0);}

	 	if(graus == 90){
			apaga_tela();
			gfx_mono_draw_string("90", 50, 12, &sysfont);
			xQueueSendFromISR(xQueueSteps, &passos, 0);}
    } 
}
}

static void task_motor(void *pvParameters) { 
 	int16_t passos = 0;
  for (;;) {
    if (xQueueReceive(xQueueSteps, &passos, 10)) {
      int passos = passos;
	//   RTT_init(1, 5000000, RTT_MR_ALMIEN);
	//   for (int i = 0; i < passos; i++) {
    // 	if ( xSemaphoreTake(xSemaphoreRTT, 0) == pdTRUE ) {			
    // } 
// }
}}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

void io_init(void) {
  pmc_enable_periph_clk(BUT_1_PIO_ID);
  pmc_enable_periph_clk(BUT_2_PIO_ID);
  pmc_enable_periph_clk(BUT_3_PIO_ID);

  pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(IN_1_PIO, PIO_OUTPUT_0, IN_1_IDX_MASK, PIO_DEFAULT);
  pio_configure(IN_2_PIO, PIO_OUTPUT_0, IN_2_IDX_MASK, PIO_DEFAULT);
  pio_configure(IN_3_PIO, PIO_OUTPUT_0, IN_3_IDX_MASK, PIO_DEFAULT);
  pio_configure(IN_4_PIO, PIO_OUTPUT_0, IN_4_IDX_MASK, PIO_DEFAULT);


  pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
  but_oled1_callback);
  pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
  but_oled2_callback);
  pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
  but_oled3_callback);

  pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
  pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
  pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

  pio_get_interrupt_status(BUT_1_PIO);
  pio_get_interrupt_status(BUT_2_PIO);
  pio_get_interrupt_status(BUT_3_PIO);

  NVIC_EnableIRQ(BUT_1_PIO_ID);
  NVIC_SetPriority(BUT_1_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_2_PIO_ID);
  NVIC_SetPriority(BUT_2_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_3_PIO_ID);
  NVIC_SetPriority(BUT_3_PIO_ID, 4);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();
	gfx_mono_ssd1306_init();

	/* Initialize the console uart */
	configure_console();
    xQueueModo = xQueueCreate(32, sizeof(int16_t));
	xQueueSteps = xQueueCreate(32, sizeof(uint32_t));
	xSemaphoreRTT = xSemaphoreCreateBinary();

	/* Create task to control oled */
	if (xTaskCreate(task_modo, "oled", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create modo task\r\n");
	}
	if (xTaskCreate(task_motor, "oled", TASK_MOTOR_STACK_SIZE, NULL, TASK_MOTOR_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create motor task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
