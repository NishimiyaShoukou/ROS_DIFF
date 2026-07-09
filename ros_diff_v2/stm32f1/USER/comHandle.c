/******************************************************************************
 * @file    comHandle.c
 * @brief   Communication command parsing and data handling routines.
 * @author  User
 * @date    2026-05-19
 * @version V2.1.0
 * @note    Part of the STM32F1 robot control project.
 *          v2.1: Battery telemetry appended to the CRC-protected uplink frame.
 ******************************************************************************/


#include "comHandle.h"
#include "usart.h"
#include "pid_debug.h"

/* ===== TX ring buffer ===== */
#define USART_TX_RING_SIZE   256U

typedef struct
{
    uint8_t  buf[USART_TX_RING_SIZE];
    volatile uint16_t head;   /* write index, advanced by producer */
    volatile uint16_t tail;   /* read  index, advanced by TXE IRQ  */
} usart_tx_ring_t;

static usart_tx_ring_t g_tx_ring;

/* ==== existing command/parse plumbing (unchanged) ==== */
union sendData
{
	short d;
	unsigned char data[2];
}left_vel_now,right_vel_now,angle_now;
union recData
{
	short d;
	unsigned char data[2];
}left_vel_get,right_vel_get,angle_get;

static com_data_t g_upper_rec_data;
static volatile short g_upper_pending_speed[2];
static volatile short g_upper_pending_angle;
static volatile uint8_t g_upper_pending_valid;

#define UART_RX_HEAD1      0x55U
#define UART_RX_HEAD2      0xaaU
#define UART_FRAME_TYPE    0x05U
#define UART_CRC_OFFSET    2U
#define UART_RX_CRC_DATA_LEN  8U
#define UART_TX_CRC_DATA_LEN  20U
#define UART_RX_FRAME_LEN  11U
#define UART_TX_FRAME_LEN  25U

typedef enum
{
    UART_RX_WAIT_HEAD1 = 0,
    UART_RX_WAIT_HEAD2,
    UART_RX_RECV_BODY,
} uart_rx_state_t;

/* ===== helpers ===== */
static __inline uint16_t usart_tx_ring_space(const usart_tx_ring_t *r)
{
    return (uint16_t)(USART_TX_RING_SIZE - (uint16_t)(r->head - r->tail));
}

static __inline uint8_t usart_tx_ring_empty(const usart_tx_ring_t *r)
{
    return (r->head == r->tail) ? 1U : 0U;
}

static void usart_tx_kick(void)
{
    /* Enable TXE interrupt; the IRQ will drain the ring one byte per call. */
    USART2->CR1 |= USART_CR1_TXEIE;
}

/*==========================================================================
 * TX ring producer: must be safe to call from main loop and from ISR.
 *==========================================================================*/
void usart_tx(uint8_t * data, uint8_t len)
{
    uint16_t space;
    uint32_t primask;
    uint8_t need_kick = 0;

    if ((data == 0) || (len == 0U))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    space = usart_tx_ring_space(&g_tx_ring);
    if (space < len)
    {
        /* Never enqueue a partial frame: it would corrupt stream alignment. */
        __set_PRIMASK(primask);
        return;
    }

    /* If ring was empty before this write, IRQ may not be running; we will
     * need to re-kick it after enabling IRQs again. */
    need_kick = usart_tx_ring_empty(&g_tx_ring);

    while (len > 0U)
    {
        g_tx_ring.buf[g_tx_ring.head % USART_TX_RING_SIZE] = *data++;
        g_tx_ring.head = (uint16_t)(g_tx_ring.head + 1U);
        len--;
        space--;
    }

    __set_PRIMASK(primask);

    if (need_kick)
    {
        usart_tx_kick();
    }
}

/*==========================================================================
 * TX ring consumer: called from USART2 TXE IRQ.
 *==========================================================================*/
void usart_tx_isr(void)
{
    if (usart_tx_ring_empty(&g_tx_ring))
    {
        /* Nothing more to send: disable TXE to avoid spurious IRQ. */
        USART2->CR1 &= ~USART_CR1_TXEIE;
        return;
    }

    USART2->DR = g_tx_ring.buf[g_tx_ring.tail % USART_TX_RING_SIZE];
    g_tx_ring.tail = (uint16_t)(g_tx_ring.tail + 1U);
}

/*==========================================================================
 * CRC-8/MAXIM: poly=0x31 (reflected 0x8c), init=0x00.
 *==========================================================================*/
uint8_t com_crc8_maxim(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0U;
    uint8_t bit;

    while (len-- > 0U)
    {
        crc ^= *data++;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x01U) ? (uint8_t)((crc >> 1) ^ 0x8cU)
                                : (uint8_t)(crc >> 1);
        }
    }
    return crc;
}

/*==========================================================================
 * RX parser: 0x55 0xaa + type/data/flag + CRC-8/MAXIM.
 *==========================================================================*/
static void usart_data_parse(const uint8_t *data_buff)
{
    left_vel_get.data[0] = data_buff[3];
    left_vel_get.data[1] = data_buff[4];

    right_vel_get.data[0] = data_buff[5];
    right_vel_get.data[1] = data_buff[6];

    angle_get.data[0] = data_buff[7];
    angle_get.data[1] = data_buff[8];

    g_upper_pending_speed[0] = left_vel_get.d;
    g_upper_pending_speed[1] = right_vel_get.d;
    g_upper_pending_angle = angle_get.d;
    g_upper_pending_valid = 1;
}

uint8_t com_take_received_data(com_data_t *out)
{
    uint8_t has_data;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    has_data = g_upper_pending_valid;
    if (has_data)
    {
        g_upper_rec_data.speed[0] = g_upper_pending_speed[0];
        g_upper_rec_data.speed[1] = g_upper_pending_speed[1];
        g_upper_rec_data.angel = g_upper_pending_angle;
        g_upper_pending_valid = 0;
        if (out != 0)
        {
            *out = g_upper_rec_data;
        }
    }

    __set_PRIMASK(primask);

    return has_data;
}

void usart_rx_to_buf(uint8_t data)
{
    static uart_rx_state_t state = UART_RX_WAIT_HEAD1;
    static uint8_t recv_index = 0;
    static uint8_t rx_value_buff[UART_RX_FRAME_LEN];

    pid_debug_rx_byte(data);

    switch (state)
    {
    case UART_RX_WAIT_HEAD1:
        if (data == UART_RX_HEAD1)
        {
            rx_value_buff[0] = data;
            recv_index = 1;
            state = UART_RX_WAIT_HEAD2;
        }
        break;

    case UART_RX_WAIT_HEAD2:
        if (data == UART_RX_HEAD2)
        {
            rx_value_buff[recv_index++] = data;
            state = UART_RX_RECV_BODY;
        }
        else if (data == UART_RX_HEAD1)
        {
            rx_value_buff[0] = data;
            recv_index = 1;
        }
        else
        {
            recv_index = 0;
            state = UART_RX_WAIT_HEAD1;
        }
        break;

    case UART_RX_RECV_BODY:
        rx_value_buff[recv_index++] = data;
        if (recv_index >= UART_RX_FRAME_LEN)
        {
            const uint8_t expected_crc =
                com_crc8_maxim(&rx_value_buff[UART_CRC_OFFSET], UART_RX_CRC_DATA_LEN);

            if ((rx_value_buff[2] == UART_FRAME_TYPE) &&
                (rx_value_buff[UART_RX_FRAME_LEN - 1U] == expected_crc))
            {
                usart_data_parse(rx_value_buff);
            }
            recv_index = 0;
            state = UART_RX_WAIT_HEAD1;
        }
        break;

    default:
        recv_index = 0;
        state = UART_RX_WAIT_HEAD1;
        break;
    }
}

static uint8_t tx_buff[UART_TX_FRAME_LEN];

static void write_int32_le(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw & 0xffU);
    data[1] = (uint8_t)((raw >> 8) & 0xffU);
    data[2] = (uint8_t)((raw >> 16) & 0xffU);
    data[3] = (uint8_t)((raw >> 24) & 0xffU);
}

static void write_uint16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xffU);
    data[1] = (uint8_t)((value >> 8) & 0xffU);
}

void usart_data_up(short left_speed_scaled, short right_speed_scaled,
                   int32_t left_encoder_count, int32_t right_encoder_count,
                   short yaw_scaled, unsigned char status,
                   uint16_t battery_voltage_mv, uint8_t battery_percent,
                   uint8_t battery_state)
{
	left_vel_now.d = left_speed_scaled;
	right_vel_now.d = right_speed_scaled;
	angle_now.d = yaw_scaled;
	tx_buff[0] = 0x55;
	tx_buff[1] = 0xaa;
	tx_buff[2] = UART_FRAME_TYPE;
	tx_buff[3] = left_vel_now.data[0];

	tx_buff[4] = left_vel_now.data[1];

	tx_buff[5] = right_vel_now.data[0];

	tx_buff[6] = right_vel_now.data[1];


	write_int32_le(&tx_buff[7], left_encoder_count);
	write_int32_le(&tx_buff[11], right_encoder_count);

	tx_buff[15] = angle_now.data[0];
	tx_buff[16] = angle_now.data[1];
	tx_buff[17] = status;
	write_uint16_le(&tx_buff[18], battery_voltage_mv);
	tx_buff[20] = battery_percent;
	tx_buff[21] = battery_state;
	tx_buff[22] = com_crc8_maxim(&tx_buff[UART_CRC_OFFSET], UART_TX_CRC_DATA_LEN);
	tx_buff[23] = 0x0d;
	tx_buff[24] = 0x0a;
	usart_tx(tx_buff, UART_TX_FRAME_LEN);

}
