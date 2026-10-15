#include "AxisCan.h"

#if !defined(STM32F1xx)
#error "AxisCan programs STM32F1 bxCAN registers directly"
#endif

// APB1 on this part is 36 MHz (72 MHz core, APB1 prescaler 2).
static const uint32_t CAN_APB1_HZ = 36000000UL;
static const uint32_t TQ_PER_BIT  = 18;   // SYNC 1 + TS1 13 + TS2 4
static const uint32_t TS1_TQ      = 13;
static const uint32_t TS2_TQ      = 4;
static const uint32_t SJW_TQ      = 1;

bool AxisCan::init(uint32_t bitrate_hz, bool loopback) {
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_CAN1_CLK_ENABLE();

  // PA11 = CAN_RX -> input with pull-up; PA12 = CAN_TX -> AF push-pull 50 MHz.
  // CRH nibble 3 is PA11, nibble 4 is PA12.
  GPIOA->CRH = (GPIOA->CRH & ~0x000FF000UL) | 0x0008B000UL;
  GPIOA->ODR |= (1UL << 11);                       // pull-up on RX

  CAN1->MCR &= ~CAN_MCR_SLEEP;                     // leave sleep
  CAN1->MCR |= CAN_MCR_INRQ;                       // request init
  for (uint32_t i = 0; i < 1000000; i++)
    if (CAN1->MSR & CAN_MSR_INAK) goto in_init;
  return false;                                    // never entered init
in_init:

  // No auto-bus-off recovery surprises; keep TX order deterministic.
  CAN1->MCR &= ~(CAN_MCR_TTCM | CAN_MCR_ABOM | CAN_MCR_AWUM |
                 CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP);

  uint32_t prescaler = CAN_APB1_HZ / (bitrate_hz * TQ_PER_BIT);
  if (prescaler < 1) prescaler = 1;
  CAN1->BTR = ((SJW_TQ - 1) << 24) |
              ((TS2_TQ - 1) << 20) |
              ((TS1_TQ - 1) << 16) |
              ((prescaler - 1) & 0x3FF) |
              (loopback ? CAN_BTR_LBKM : 0);

  CAN1->MCR &= ~CAN_MCR_INRQ;                      // leave init, go normal
  for (uint32_t i = 0; i < 1000000; i++)
    if (!(CAN1->MSR & CAN_MSR_INAK)) goto running;
  return false;
running:

  // One filter, accept everything, into FIFO 0. Addressing is done in software
  // so extra axes can be added without touching filter banks.
  CAN1->FMR  |= CAN_FMR_FINIT;
  CAN1->FA1R &= ~1UL;                              // deactivate bank 0
  CAN1->FS1R |= 1UL;                               // 32-bit scale
  CAN1->FM1R &= ~1UL;                              // mask mode
  CAN1->sFilterRegister[0].FR1 = 0x00000000;       // id
  CAN1->sFilterRegister[0].FR2 = 0x00000000;       // mask 0 = don't care
  CAN1->FFA1R &= ~1UL;                             // FIFO 0
  CAN1->FA1R |= 1UL;                               // activate
  CAN1->FMR  &= ~CAN_FMR_FINIT;

  tx_ = rx_ = 0;
  return true;
}

bool AxisCan::send(uint16_t id, const void* data, uint8_t len) {
  if (len > 8) len = 8;
  int box = -1;
  if (CAN1->TSR & CAN_TSR_TME0) box = 0;
  else if (CAN1->TSR & CAN_TSR_TME1) box = 1;
  else if (CAN1->TSR & CAN_TSR_TME2) box = 2;
  if (box < 0) return false;                       // all mailboxes busy

  const uint8_t* p = (const uint8_t*)data;
  uint32_t lo = 0, hi = 0;
  for (uint8_t i = 0; i < len; i++) {
    if (i < 4) lo |= (uint32_t)p[i] << (8 * i);
    else       hi |= (uint32_t)p[i] << (8 * (i - 4));
  }
  CAN1->sTxMailBox[box].TDLR = lo;
  CAN1->sTxMailBox[box].TDHR = hi;
  CAN1->sTxMailBox[box].TDTR = len;                // standard id, data frame
  CAN1->sTxMailBox[box].TIR  = ((uint32_t)(id & 0x7FF) << 21) | CAN_TI0R_TXRQ;
  tx_++;
  return true;
}

bool AxisCan::receive(CanFrame* out) {
  if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0) return false;
  out->id  = (uint16_t)((CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF);
  out->len = (uint8_t)(CAN1->sFIFOMailBox[0].RDTR & 0x0F);
  if (out->len > 8) out->len = 8;
  uint32_t lo = CAN1->sFIFOMailBox[0].RDLR;
  uint32_t hi = CAN1->sFIFOMailBox[0].RDHR;
  for (uint8_t i = 0; i < out->len; i++)
    out->data[i] = (i < 4) ? (uint8_t)(lo >> (8 * i)) : (uint8_t)(hi >> (8 * (i - 4)));
  CAN1->RF0R |= CAN_RF0R_RFOM0;                    // release the mailbox
  rx_++;
  return true;
}

uint32_t AxisCan::errCount() const {
  return ((CAN1->ESR >> 16) & 0xFF) + ((CAN1->ESR >> 24) & 0xFF);  // TEC + REC
}
uint8_t  AxisCan::lastError() const { return (uint8_t)((CAN1->ESR >> 4) & 0x7); }
uint32_t AxisCan::btr() const { return CAN1->BTR; }
