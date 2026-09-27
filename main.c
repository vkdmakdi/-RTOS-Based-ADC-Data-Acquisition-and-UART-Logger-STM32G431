#include "stm32g4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "FreeRTOSConfig.h"
#include <stdint.h>

#define RCC_REG_BASE            0x40021000U
#define RCC_AHB1ENR_REG     (*(volatile uint32_t *)(RCC_REG_BASE + 0x48U))
#define RCC_AHB2ENR_REG     (*(volatile uint32_t *)(RCC_REG_BASE + 0x4CU))
#define RCC_APB1ENR1_REG    (*(volatile uint32_t *)(RCC_REG_BASE + 0x58U))
#define RCC_APB2ENR_REG     (*(volatile uint32_t *)(RCC_REG_BASE + 0x60U))
#define RCC_CCIPR_REG       (*(volatile uint32_t *)(RCC_REG_BASE + 0x88U))

#define GPIOA_REG_BASE          0x48000000U
#define GPIOA_MODE_REG      (*(volatile uint32_t *)(GPIOA_REG_BASE + 0x00U))
#define GPIOA_AFRH_REG      (*(volatile uint32_t *)(GPIOA_REG_BASE + 0x24U))
#define GPIOA_ASCR_REG      (*(volatile uint32_t *)(GPIOA_REG_BASE + 0x2CU))

#define USART1_REG_BASE         0x40013800U
#define USART1_CR1_REG      (*(volatile uint32_t *)(USART1_REG_BASE + 0x00U))
#define USART1_BRR_REG      (*(volatile uint32_t *)(USART1_REG_BASE + 0x0CU))
#define USART1_ISR_REG      (*(volatile uint32_t *)(USART1_REG_BASE + 0x1CU))
#define USART1_TDR_REG      (*(volatile uint32_t *)(USART1_REG_BASE + 0x28U))

#define TIM2_REG_BASE           0x40000000U
#define TIM2_CR1_REG        (*(volatile uint32_t *)(TIM2_REG_BASE + 0x00U))
#define TIM2_CR2_REG        (*(volatile uint32_t *)(TIM2_REG_BASE + 0x04U))
#define TIM2_PSC_REG        (*(volatile uint32_t *)(TIM2_REG_BASE + 0x28U))
#define TIM2_ARR_REG        (*(volatile uint32_t *)(TIM2_REG_BASE + 0x2CU))

#define ADC1_REG_BASE           0x50000000UL
#define ADC1_ISR_REG        (*(volatile uint32_t *)(ADC1_REG_BASE + 0x00U))
#define ADC1_CR_REG         (*(volatile uint32_t *)(ADC1_REG_BASE + 0x08U))
#define ADC1_CFGR_REG       (*(volatile uint32_t *)(ADC1_REG_BASE + 0x0CU))
#define ADC1_SQR1_REG       (*(volatile uint32_t *)(ADC1_REG_BASE + 0x30U))
#define ADC1_DR_REG         (*(volatile uint32_t *)(ADC1_REG_BASE + 0x40U))

#define DMA1_REG_BASE           0x40020000U
#define DMAMUX1_REG_BASE        0x40020800U
#define DMA1_ISR_REG        (*(volatile uint32_t *)(DMA1_REG_BASE + 0x00U))
#define DMA1_IFCR_REG       (*(volatile uint32_t *)(DMA1_REG_BASE + 0x04U))
#define DMA1_CCR1_REG       (*(volatile uint32_t *)(DMA1_REG_BASE + 0x08U))
#define DMA1_CNDTR1_REG     (*(volatile uint32_t *)(DMA1_REG_BASE + 0x0CU))
#define DMA1_CPAR1_REG      (*(volatile uint32_t *)(DMA1_REG_BASE + 0x10U))
#define DMA1_CMAR1_REG      (*(volatile uint32_t *)(DMA1_REG_BASE + 0x14U))
#define DMAMUX1_C0CR_REG    (*(volatile uint32_t *)(DMAMUX1_REG_BASE + 0x00U))

#define PACKET_SYNC         '#'
#define PAYLOAD_SIZE        8U
#define PACKET_SIZE         (1U + PAYLOAD_SIZE + 1U)
#define ADC_BUFFER_LENGTH   8U
#define PACKETS_PER_QUEUE   8U
#define ADC_EVENT_HALF      (1UL << 0)
#define ADC_EVENT_FULL      (1UL << 1)
#define ADC_EVENT_MASK      (ADC_EVENT_HALF | ADC_EVENT_FULL)


/* One ADC channel sampled at 1 kHz; each packet contains four consecutive samples. */
static volatile uint16_t adc_ring_buffer[ADC_BUFFER_LENGTH];

typedef struct
{
    uint16_t samples[4];
} AdcPacket_t;

static QueueHandle_t packetQueue;
TaskHandle_t adcTaskHandle = NULL;
static volatile uint32_t droppedPacketCount;
volatile uint32_t dmaErrorCount;

void Error_Handler(void);
static void system_init(void);
static void adc_calibrate(void);
static void acquisition_task(void *argument);
static void uart_task(void *argument);
static void send_packet(const AdcPacket_t *packet);
static void uart_write_byte(uint8_t byte);

int main(void)
{
    system_init();

    packetQueue = xQueueCreate(PACKETS_PER_QUEUE, sizeof(AdcPacket_t));
    if (packetQueue == NULL)
    {
        Error_Handler();
    }

    if (xTaskCreate(acquisition_task, "adc", 256U, NULL,
                    tskIDLE_PRIORITY + 3U, &adcTaskHandle) != pdPASS)
    {
        Error_Handler();
    }

    if (xTaskCreate(uart_task, "uart", 256U, NULL,
                    tskIDLE_PRIORITY + 2U, NULL) != pdPASS)
    {
        Error_Handler();
    }

    vTaskStartScheduler();
    Error_Handler();
    return 0;
}

static void acquisition_task(void *argument)
{
    (void) argument;

    /* Start sampling only after the task handle is ready for DMA notifications. */
    TIM2_CR1_REG |= (1UL << 0);

    for (;;)
    {
        uint32_t events = 0U;
        (void) xTaskNotifyWait(0U, ADC_EVENT_MASK, &events, portMAX_DELAY);

        if ((events & ADC_EVENT_HALF) != 0U)
        {
            AdcPacket_t packet;
            for (uint32_t i = 0U; i < 4U; ++i)
            {
                packet.samples[i] = adc_ring_buffer[i];
            }
            if (xQueueSend(packetQueue, &packet, 0U) != pdPASS)
            {
                ++droppedPacketCount;
            }
        }

        if ((events & ADC_EVENT_FULL) != 0U)
        {
            AdcPacket_t packet;
            for (uint32_t i = 0U; i < 4U; ++i)
            {
                packet.samples[i] = adc_ring_buffer[i + 4U];
            }
            if (xQueueSend(packetQueue, &packet, 0U) != pdPASS)
            {
                ++droppedPacketCount;
            }
        }
    }
}

static void uart_task(void *argument)
{
    (void) argument;
    AdcPacket_t packet;

    for (;;)
    {
        if (xQueueReceive(packetQueue, &packet, portMAX_DELAY) == pdPASS)
        {
            send_packet(&packet);
        }
    }
}

static void uart_write_byte(uint8_t byte)
{
    while ((USART1_ISR_REG & USART_ISR_TXE) == 0U)
    {

    }
    USART1_TDR_REG = byte;
}

static void send_packet(const AdcPacket_t *packet)
{
    uint8_t txBuffer[PACKET_SIZE];
    uint8_t checksum = 0U;
    uint32_t byteIndex = 1U;

    txBuffer[0] = (uint8_t) PACKET_SYNC;

    for (uint32_t i = 0U; i < 4U; ++i)
    {
        const uint16_t sample = packet->samples[i];
        const uint8_t lowByte = (uint8_t) (sample & 0xFFU);
        const uint8_t highByte = (uint8_t) (sample >> 8U);

        txBuffer[byteIndex++] = lowByte;
        txBuffer[byteIndex++] = highByte;
        checksum ^= lowByte;
        checksum ^= highByte;
    }

    txBuffer[PACKET_SIZE - 1U] = checksum;
    for (uint32_t i = 0U; i < PACKET_SIZE; ++i)
    {
        uart_write_byte(txBuffer[i]);
    }
}

static void adc_calibrate(void)
{
    ADC1_CR_REG &= ~(1UL << 29U); /* DEEPPWD = 0 */
    ADC1_CR_REG |=  (1UL << 28U); /* ADVREGEN = 1 */
    for (volatile uint32_t i = 0U; i < 1000U; ++i) { }

    ADC1_CR_REG |= (1UL << 31U); /* ADCAL = 1 */
    while ((ADC1_CR_REG & (1UL << 31U)) != 0U) { }
}

static void system_init(void)
{
    /* System clock remains the reset HSI clock (16 MHz). */
    RCC_AHB1ENR_REG |= (1UL << 0U);   /* DMA1 */
    RCC_AHB2ENR_REG |= (1UL << 0U);   /* GPIOA */
    RCC_AHB2ENR_REG |= (1UL << 13U);  /* ADC12 */
    RCC_APB1ENR1_REG |= (1UL << 0U);  /* TIM2 */
    RCC_APB2ENR_REG |= (1UL << 14U);  /* USART1 */

    /* ADC12SEL=2 selects SYSCLK; 3 is reserved on STM32G4. */
    RCC_CCIPR_REG = (RCC_CCIPR_REG & ~(3UL << 28U)) | (2UL << 28U);

    /* PA9 = USART1_TX (AF7); PA10 = USART1_RX (AF7). */
    GPIOA_MODE_REG = (GPIOA_MODE_REG & ~((3UL << 18U) | (3UL << 20U)))
                   | ((2UL << 18U) | (2UL << 20U));
    GPIOA_AFRH_REG = (GPIOA_AFRH_REG & ~((15UL << 4U) | (15UL << 8U)))
                   | ((7UL << 4U) | (7UL << 8U));

    /* PA0 = ADC1 channel 1. */
    GPIOA_MODE_REG = (GPIOA_MODE_REG & ~(3UL << 0U)) | (3UL << 0U);
    GPIOA_ASCR_REG |= (1UL << 0U);

    /* USART1 at approximately 115200 baud from the 16 MHz HSI clock. */
    USART1_BRR_REG = 139U;
    USART1_CR1_REG |= (1UL << 3U) | (1UL << 0U); /* TE | UE */

    /* TIM2 update at 1 kHz: 16 MHz / 16 / 1000. */
    TIM2_PSC_REG = 16U - 1U;
    TIM2_ARR_REG = 1000U - 1U;
    TIM2_CR2_REG = (TIM2_CR2_REG & ~(7UL << 4U)) | (2UL << 4U); /* TRGO=update */
    TIM2_CR1_REG &= ~(1UL << 0U); /* Start in acquisition_task. */

    /* ADC DMA circular buffer: four samples per half-buffer. */
    DMA1_CCR1_REG &= ~(1UL << 0U);
    DMA1_CPAR1_REG = (uint32_t) &ADC1_DR_REG;
    DMA1_CMAR1_REG = (uint32_t) adc_ring_buffer;
    DMA1_CNDTR1_REG = ADC_BUFFER_LENGTH;
    DMA1_IFCR_REG = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
    DMA1_CCR1_REG = (1UL << 8U)  /* PSIZE=16 bit */
                  | (1UL << 10U) /* MSIZE=16 bit */
                  | (1UL << 7U)  /* MINC */
                  | (1UL << 5U)  /* CIRC */
                  | (1UL << 1U)  /* TCIE */
                  | (1UL << 2U); /* HTIE */
    DMAMUX1_C0CR_REG = 5U; /* ADC1 request */

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    NVIC_SetPriority(DMA1_Channel1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* ADC: single regular conversion on channel 1, triggered by TIM2_TRGO. */
    adc_calibrate();
    ADC1_CFGR_REG &= ~((31UL << 5U) | (3UL << 10U) | (1UL << 13U)
                     | (1UL << 0U) | (1UL << 1U));
    ADC1_CFGR_REG |= (11UL << 5U) | (1UL << 10U) | (1UL << 0U) | (1UL << 1U);
    ADC1_SQR1_REG = (ADC1_SQR1_REG & ~((15UL << 0U) | (0x1FUL << 6U)))
                  | (1UL << 6U);

    ADC1_CR_REG |= (1UL << 0U); /* ADEN */
    while ((ADC1_ISR_REG & (1UL << 0U)) == 0U) { } /* ADRDY */
    ADC1_ISR_REG = (1UL << 0U); /* Clear ADRDY. */

    DMA1_CCR1_REG |= (1UL << 0U); /* Enable DMA before ADC conversions. */
    ADC1_CR_REG |= (1UL << 2U);   /* ADSTART; TIM2 triggers conversions. */
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *taskName)
{
    (void) task;
    (void) taskName;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void Error_Handler(void)
{
    __disable_irq();
    for (;;) { }
}
