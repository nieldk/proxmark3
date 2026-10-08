//
// Created by dxl on 2026/5/23.
//

#include "commonutil.h"
#include "sys_apis.h"
#include "config_gpio_proxmark5.h"

// --- at32 ---
#include "at32f435_437_crm.h"
#include "at32f435_437_pwc.h"
#include "at32f435_437_flash.h"
#include "at32f435_437_misc.h"
#include "at32f435_437_ertc.h"
// ---

#define SYS_SIMPLE_RESET_BPR_MAGIC            0x504D3352U
#define SYS_SIMPLE_RESET_BPR_UNLOCK_KEY1      0xCAU
#define SYS_SIMPLE_RESET_BPR_UNLOCK_KEY2      0x53U
#define SYS_SIMPLE_RESET_BPR_LOCK_KEY         0xFFU

uint8_t g_system_reset_method = 0; // Default set to 0 for call system_simple_reset() + jump bootrom.

/**
 * @brief empty call definition, avoid errors linking libc.a
 * @param fn_name function name to define for libc.a
 */
#define EMPTY_CALL(fn_name)      \
    void fn_name(void);          \
    __WEAK void fn_name(void) {}

// For simple reset & jump to bootrom restart the device.
extern uint32_t _bootrom_start[], _stack_end[];

// Empty init definition to avoid errors linking libc.a
EMPTY_CALL(_init)

// Empty _fini definition to avoid errors linking libc.a
EMPTY_CALL(_fini)

/**
 * Write data to BPR(Battery powered domain data) register 1
 * @param data The data to write to the BPR register 1
 *
 * The ERTC write-protection register (ERTC->wp) lives in the ERTC's own clock
 * domain. This code enabled the ERTC (ertcen) but never selected an ERTC clock
 * source (ertcsel stays NOCLK), so writing ERTC->wp stalled the APB once the CPU
 * ran at the full PLL clock (288MHz, 144MHz APB) -- it only "worked" at the slow
 * HICK clock. In the bootrom this runs after ConfigSystemClocks() (the button
 * path in check_goto_flash_mode() needs SpinDelayUs()/the timer clock, so the
 * clock must be up first), i.e. at 288MHz, which hung the "enter flash mode"
 * magic handling on the warm software-reset path: USB never re-enumerated into
 * the bootloader until a physical replug.
 *
 * Fix: give the ERTC a clock source before touching its registers, the same one
 * the tick HAL uses (HEXT/20). HEXT is running by the time the bootrom reaches
 * this (after ConfigSystemClocks), so the ERTC clock is live and the wp write no
 * longer stalls. We do NOT reset the battery-powered domain -- that would wipe
 * the BPR magic itself -- and we do NOT wait on the calendar-update flag (that
 * would spin to its timeout on paths where HEXT is off, e.g. system_simple_reset,
 * which still write correctly because they run on the slow HICK clock). ertcsel
 * is write-once until a domain reset, so if it is already selected this is a
 * no-op.
 */
static void at32_bpr_write_dt1(uint32_t data) {
    CRM->apb1en_bit.pwcen = TRUE;
    PWC->ctrl_bit.bpwen = TRUE;

    // Select and enable an ERTC clock so the write-protection register is
    // writable at the full APB frequency.
    CRM->cfg_bit.ertcdiv = ((CRM_ERTC_CLOCK_HEXT_DIV_20 & 0x1F0) >> 4);
    CRM->bpdc_bit.ertcsel = (CRM_ERTC_CLOCK_HEXT_DIV_20 & 0xF);
    CRM->bpdc_bit.ertcen = TRUE;

    ERTC->wp = SYS_SIMPLE_RESET_BPR_UNLOCK_KEY1;
    ERTC->wp = SYS_SIMPLE_RESET_BPR_UNLOCK_KEY2;
    ERTC->dt1 = data;
    ERTC->wp = SYS_SIMPLE_RESET_BPR_LOCK_KEY;

    CRM->apb1en_bit.pwcen = FALSE;
    PWC->ctrl_bit.bpwen = FALSE;
    CRM->bpdc_bit.ertcen = FALSE;
}

/**
 * @brief  check BPR register 1, if it is equal to SYS_SIMPLE_RESET_BPR_MAGIC,
 *  clear it and return true, otherwise return false.
 * @return true if the BPR register 1 is equal to SYS_SIMPLE_RESET_BPR_MAGIC, false otherwise
 */
bool system_bpr_chk_clear(void) {
    CRM->apb1en_bit.pwcen = TRUE;
    PWC->ctrl_bit.bpwen = TRUE;
    CRM->bpdc_bit.ertcen = TRUE;

    if (ERTC->dt1 == SYS_SIMPLE_RESET_BPR_MAGIC) {
        at32_bpr_write_dt1(0);
        return true;
    }
    return false;
}

/**
  * @brief  this function handles nmi exception.
  * @retval none
  */
void NMI_Handler(void) {
}

#ifdef AS_BOOTROM
// Bootrom: a restart would re-enter the bootrom, keep the plain halt.
void HardFault_Handler(void) {
    while (1) {
    }
}

void MemManage_Handler(void) {
    while (1) {
    }
}

void BusFault_Handler(void) {
    while (1) {
    }
}

void UsageFault_Handler(void) {
    while (1) {
    }
}
#else

// Fault record, kept in the battery domain registers dt2..dt7 across the restart.
// dt7 is written last and holds the tag, so a half-written record is ignored.
#define SYS_FAULT_TAG       0xFA170000U
#define SYS_FAULT_TAG_MASK  0xFFFF0000U

static sys_fault_record_t s_fault_rec;

// Reached after the fault handler returned into thread mode.
static void __NO_RETURN fault_restart(void) {
    system_simple_reset();
    while (1) {
    }
}

static void at32_bpr_write_fault(uint32_t pc, uint32_t lr, uint32_t cfsr, uint32_t addr, uint32_t hfsr, uint32_t vect) {
    CRM->apb1en_bit.pwcen = TRUE;
    PWC->ctrl_bit.bpwen = TRUE;
    CRM->cfg_bit.ertcdiv = ((CRM_ERTC_CLOCK_HEXT_DIV_20 & 0x1F0) >> 4);
    CRM->bpdc_bit.ertcsel = (CRM_ERTC_CLOCK_HEXT_DIV_20 & 0xF);
    CRM->bpdc_bit.ertcen = TRUE;

    ERTC->wp = SYS_SIMPLE_RESET_BPR_UNLOCK_KEY1;
    ERTC->wp = SYS_SIMPLE_RESET_BPR_UNLOCK_KEY2;
    ERTC->dt2 = pc;
    ERTC->dt3 = lr;
    ERTC->dt4 = cfsr;
    ERTC->dt5 = addr;
    ERTC->dt6 = hfsr;
    ERTC->dt7 = SYS_FAULT_TAG | (vect & 0x1FFU);
    ERTC->wp = SYS_SIMPLE_RESET_BPR_LOCK_KEY;
}

void sys_fault_c(uint32_t *frame, uint32_t exc_ret);

// frame: stacked r0-r3,r12,lr,pc,xpsr. exc_ret: EXC_RETURN value.
// Returning from here resumes at fault_restart() in thread mode.
void sys_fault_c(uint32_t *frame, uint32_t exc_ret) {
    uint32_t pc = 0xFFFFFFFFU, lr = 0xFFFFFFFFU;
    bool frame_ok = ((uint32_t)frame >= 0x20000000U) && ((uint32_t)frame <= 0x200FFFE0U) && (((uint32_t)frame & 3U) == 0);
    if (frame_ok) {
        lr = frame[5];
        pc = frame[6];
    }

    uint32_t cfsr = SCB->CFSR;
    uint32_t addr = (cfsr & 0x8000U) ? SCB->BFAR : ((cfsr & 0x80U) ? SCB->MMFAR : 0);
    uint32_t ipsr;
    __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));

    uint32_t hfsr = SCB->HFSR;
    SCB->CFSR = cfsr;   // write 1 to clear, else stale bits show up in the next record
    SCB->HFSR = hfsr;

    at32_bpr_write_fault(pc, lr, cfsr, addr, hfsr, ipsr);

    // Can only redirect a thread-mode fault with a usable frame. Anything else
    // (fault inside an ISR, bad stack) gets a core reset.
    if (frame_ok && (exc_ret & 0x8U)) {
        frame[5] = 0;
        frame[6] = (uint32_t)fault_restart;
        return;
    }
    nvic_system_reset();
    while (1) {
    }
}

// Pick the active stack pointer from EXC_RETURN and call sys_fault_c().
#define SYS_FAULT_TRAMPOLINE(name) \
    __asm__(".pushsection .text.sysfault_" #name ",\"ax\",%progbits\n" \
            ".syntax unified\n" \
            ".thumb\n" \
            ".global " #name "\n" \
            ".type " #name ", %function\n" \
            ".thumb_func\n" \
            #name ":\n" \
            "  tst lr, #4\n" \
            "  ite eq\n" \
            "  mrseq r0, msp\n" \
            "  mrsne r0, psp\n" \
            "  mov r1, lr\n" \
            "  push {r4, lr}\n" \
            "  bl sys_fault_c\n" \
            "  pop {r4, pc}\n" \
            ".popsection\n")

SYS_FAULT_TRAMPOLINE(HardFault_Handler);
SYS_FAULT_TRAMPOLINE(MemManage_Handler);
SYS_FAULT_TRAMPOLINE(BusFault_Handler);
SYS_FAULT_TRAMPOLINE(UsageFault_Handler);

// Copy a record left by the previous run and clear it. Call once at startup.
void sys_fault_record_capture(void) {
    CRM->apb1en_bit.pwcen = TRUE;
    PWC->ctrl_bit.bpwen = TRUE;
    CRM->bpdc_bit.ertcen = TRUE;

    uint32_t tag = ERTC->dt7;
    if ((tag & SYS_FAULT_TAG_MASK) != SYS_FAULT_TAG) {
        return;
    }
    s_fault_rec.valid = true;
    s_fault_rec.vector = tag & 0x1FFU;
    s_fault_rec.pc = ERTC->dt2;
    s_fault_rec.lr = ERTC->dt3;
    s_fault_rec.cfsr = ERTC->dt4;
    s_fault_rec.addr = ERTC->dt5;
    s_fault_rec.hfsr = ERTC->dt6;

    ERTC->wp = SYS_SIMPLE_RESET_BPR_UNLOCK_KEY1;
    ERTC->wp = SYS_SIMPLE_RESET_BPR_UNLOCK_KEY2;
    ERTC->dt7 = 0;
    ERTC->wp = SYS_SIMPLE_RESET_BPR_LOCK_KEY;
}

bool sys_fault_record_get(sys_fault_record_t *out) {
    if (!s_fault_rec.valid) {
        return false;
    }
    *out = s_fault_rec;
    return true;
}
#endif // AS_BOOTROM

/**
  * @brief  this function handles svcall exception.
  * @retval none
  */
void SVC_Handler(void) {
}

/**
  * @brief  this function handles debug monitor exception.
  * @retval none
  */
void DebugMon_Handler(void) {
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @retval none
  */
void PendSV_Handler(void) {
}

/**
  * @brief  this function handles systick handler.
  * @retval none
  */
void SysTick_Handler(void) {
}

/**
  * @brief  system clock config program
  * @note   the system clock is configured as follow:
  *         system clock (sclk)   = (hext * pll_ns)/(pll_ms * pll_fr)
  *         system clock source   = HEXT_VALUE
  *         - hext                = 8000000
  *         - sclk                = 48000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 48000000
  *         - apb1div             = 2
  *         - apb1clk             = 24000000
  *         - apb2div             = 1
  *         - apb2clk             = 48000000
  *         - pll_ns              = 96
  *         - pll_ms              = 1
  *         - pll_fr              = 16
  * @retval none
  */
void system_clock_config_48m(void) {
    /* reset crm */
    crm_reset();

    /* enable pwc periph clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

    /* config ldo voltage */
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V1);

    /* set the flash clock divider */
    flash_clock_divider_set(FLASH_CLOCK_DIV_2);

    /* enable hext */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

    /* wait till hext is ready */
    while (crm_hext_stable_wait() == ERROR) {
    }

    /* config pll clock resource
    common frequency config list: pll source selected  hick or hext(8mhz)
    _________________________________________________________________________________________________
    |        |         |         |         |         |         |         |         |        |        |
    |pll(mhz)|   288   |   252   |   216   |   192   |   180   |   144   |   108   |   72   |   36   |
    |________|_________|_________|_________|_________|_________|_________|_________|_________________|
    |        |         |         |         |         |         |         |         |        |        |
    |pll_ns  |   144   |   126   |   108   |   96    |   90    |   72    |   108   |   72   |   72   |
    |        |         |         |         |         |         |         |         |        |        |
    |pll_ms  |   1     |   1     |   1     |   1     |   1     |   1     |   1     |   1    |   1    |
    |        |         |         |         |         |         |         |         |        |        |
    |pll_fr  |   FR_4  |   FR_4  |   FR_4  |   FR_4  |   FR_4  |   FR_4  |   FR_8  |   FR_8 |   FR_16|
    |________|_________|_________|_________|_________|_________|_________|_________|________|________|

    if pll clock source selects hext with other frequency values, or configure pll to other
    frequency values, please use the at32 new clock  configuration tool for configuration.  */
    crm_pll_config(CRM_PLL_SOURCE_HEXT, 96, 1, CRM_PLL_FR_16);

    /* enable pll */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

    /* wait till pll is ready */
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {
    }

    /* config ahbclk */
    crm_ahb_div_set(CRM_AHB_DIV_1);

    /* config apb2clk */
    crm_apb2_div_set(CRM_APB2_DIV_1);

    /* config apb1clk */
    crm_apb1_div_set(CRM_APB1_DIV_2);

    /* select pll as system clock source */
    crm_sysclk_switch(CRM_SCLK_PLL);

    /* wait till pll is used as system clock source */
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
    }

    /* update system_core_clock global variable */
    system_core_clock_update();
}

/**
  * @brief  system clock config
  * @note   the system clock is configured as follow:
  *         system clock (sclk)   = (hext * pll_ns)/(pll_ms * pll_fr)
  *         system clock source   = pll (hext)
  *         - hext                = HEXT_VALUE
  *         - sclk                = 288000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 288000000
  *         - apb2div             = 2
  *         - apb2clk             = 144000000
  *         - apb1div             = 2
  *         - apb1clk             = 144000000
  *         - pll_ns              = 144
  *         - pll_ms              = 1
  *         - pll_fr              = 4
  * @retval none
  */
void system_clock_config_288m(void) {
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    /* reset crm */
    crm_reset();

    /* enable pwc periph clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

    /* config ldo voltage */
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

    /* set the flash clock divider */
    flash_clock_divider_set(FLASH_CLOCK_DIV_3);

    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

    /* wait till hext is ready */
    while (crm_hext_stable_wait() == ERROR) {
    }

    /* config pll clock resource
    common frequency config list: pll source selected  hick or hext(8mhz)
    _________________________________________________________________________________________________
    |        |         |         |         |         |         |         |         |        |        |
    |pll(mhz)|   288   |   252   |   216   |   192   |   180   |   144   |   108   |   72   |   36   |
    |________|_________|_________|_________|_________|_________|_________|_________|_________________|
    |        |         |         |         |         |         |         |         |        |        |
    |pll_ns  |   144   |   126   |   108   |   96    |   90    |   72    |   108   |   72   |   72   |
    |        |         |         |         |         |         |         |         |        |        |
    |pll_ms  |   1     |   1     |   1     |   1     |   1     |   1     |   1     |   1    |   1    |
    |        |         |         |         |         |         |         |         |        |        |
    |pll_fr  |   FR_4  |   FR_4  |   FR_4  |   FR_4  |   FR_4  |   FR_4  |   FR_8  |   FR_8 |   FR_16|
    |________|_________|_________|_________|_________|_________|_________|_________|________|________|

    if pll clock source selects hext with other frequency values, or configure pll to other
    frequency values, please use the at32 new clock  configuration tool for configuration.  */
    crm_pll_config(CRM_PLL_SOURCE_HEXT, 144, 1, CRM_PLL_FR_4);

    /* enable pll */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

    /* wait till pll is ready */
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {
    }

    /* config ahbclk */
    crm_ahb_div_set(CRM_AHB_DIV_1); // 288mhz

    /* config apb2clk, the maximum frequency of APB1/APB2 clock is 144 MHz  */
    crm_apb2_div_set(CRM_APB2_DIV_2); // 144mhz(288mhz / 2)

    /* config apb1clk, the maximum frequency of APB1/APB2 clock is 144 MHz  */
    crm_apb1_div_set(CRM_APB1_DIV_2); // 144mhz(288mhz / 2)

    /* enable auto step mode */
    crm_auto_step_mode_enable(TRUE);

    /* select pll as system clock source */
    crm_sysclk_switch(CRM_SCLK_PLL);

    /* wait till pll is used as system clock source */
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
    }

    /* disable auto step mode */
    crm_auto_step_mode_enable(FALSE);

    /* update system_core_clock global variable */
    system_core_clock_update();
}

/**
  * @brief  system clock config
  * @note   the system clock is configured as follow:
  *         system clock (sclk)   = (hext * pll_ns)/(pll_ms * pll_fr)
  *         system clock source   = pll (hext)
  *         - hext                = HEXT_VALUE
  *         - sclk                = 288000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 288000000
  *         - apb2div             = 2
  *         - apb2clk             = 144000000
  *         - apb1div             = 2
  *         - apb1clk             = 144000000
  *         - pll_ns              = 144
  *         - pll_ms              = 1
  *         - pll_fr              = 4
  * @retval none
  */
void ConfigSystemClocks(void) {
    // system_clock_config_48m();
    system_clock_config_288m();
}

/**
 * The state of GPIOB (especially PB0: arm power on) is preserved,
 * while other peripherals are reset/clock-gated as much as possible, without using NVIC reset.
 */
void system_simple_reset(void) {
    const uint32_t keep_gpiob_clock_mask = CRM_REG_BIT(AT32_GPIO_ARM_POWER_LOCK_CLK);
    uint32_t ahb1_reset_mask =
        CRM_REG_BIT(CRM_GPIOA_PERIPH_RESET) |
        CRM_REG_BIT(CRM_GPIOC_PERIPH_RESET) |
        CRM_REG_BIT(CRM_GPIOD_PERIPH_RESET) |
        CRM_REG_BIT(CRM_GPIOE_PERIPH_RESET) |
        CRM_REG_BIT(CRM_GPIOF_PERIPH_RESET) |
        CRM_REG_BIT(CRM_GPIOG_PERIPH_RESET) |
        CRM_REG_BIT(CRM_GPIOH_PERIPH_RESET) |
        CRM_REG_BIT(CRM_CRC_PERIPH_RESET) |
        CRM_REG_BIT(CRM_EDMA_PERIPH_RESET) |
        CRM_REG_BIT(CRM_DMA1_PERIPH_RESET) |
        CRM_REG_BIT(CRM_DMA2_PERIPH_RESET) |
        CRM_REG_BIT(CRM_OTGFS2_PERIPH_RESET);
#if defined(AT32F437xx)
    ahb1_reset_mask |= CRM_REG_BIT(CRM_EMAC_PERIPH_RESET);
#endif

    __disable_irq();

    /* Return to HICK first, then close PLL/HEXT and reset CRM clock tree settings. */
    CRM->ctrl_bit.hicken = TRUE;
    while (CRM->ctrl_bit.hickstbl != SET) {
    }
    CRM->cfg_bit.sclksel = CRM_SCLK_HICK;
    while (CRM->cfg_bit.sclksts != CRM_SCLK_HICK) {
    }
    CRM->ctrl &= ~(0x010D0000U);
    CRM->cfg = 0;
    CRM->pllcfg = 0x00033002U;
    CRM->misc1 = 0;
    CRM->misc2 = 0;

    /* Reset AHB peripherals directly, excluding GPIOB to keep PB0 state stable. */
    CRM->ahbrst1 = ahb1_reset_mask;
    CRM->ahbrst2 = CRM_REG_BIT(CRM_DVP_PERIPH_RESET) |
                   CRM_REG_BIT(CRM_OTGFS1_PERIPH_RESET) |
                   CRM_REG_BIT(CRM_SDIO1_PERIPH_RESET);
    CRM->ahbrst3 = CRM_REG_BIT(CRM_XMC_PERIPH_RESET) |
                   CRM_REG_BIT(CRM_QSPI1_PERIPH_RESET) |
                   CRM_REG_BIT(CRM_QSPI2_PERIPH_RESET) |
                   CRM_REG_BIT(CRM_SDIO2_PERIPH_RESET);
    CRM->ahbrst1 = 0;
    CRM->ahbrst2 = 0;
    CRM->ahbrst3 = 0;

    /* Disable all peripheral clocks directly, but keep GPIOB clock for PB0 control path. */
    CRM->ahben1 = keep_gpiob_clock_mask;
    CRM->ahben2 = 0;
    CRM->ahben3 = 0;
    CRM->apb1rst = 0xFFFF;
    CRM->apb1rst = 0;
    CRM->apb1en = 0;
    CRM->apb2rst = 0xFFFF;
    CRM->apb2rst = 0;
    CRM->apb2en = 0;
    CRM->clkint = 0x009F0000U;

    // Write BPR_1 before jump to bootrom to restart.
    at32_bpr_write_dt1(SYS_SIMPLE_RESET_BPR_MAGIC);

    // Jump to bootrom
    JumpToAnyImage((uint32_t) _stack_end, (uint32_t) _bootrom_start);
}

// Refer to the code described in the following link to implement the jump.
// https://community.st.com/t5/stm32-mcus-products/jump-to-application-from-bootloader-not-working/td-p/620734
void __NO_RETURN JumpToAnyImage(uint32_t stack_top, uint32_t entry_point) {
    // Disable and clear all pending interrupts in the Bootloader
    __disable_irq();
    for (int i = 0; i < ARRAYLEN(NVIC->ICER); i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // Disable SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    SCB->VTOR = entry_point; // Update the Vector Table Offset Register (VTOR)
    __set_MSP(stack_top); // Set the Main Stack Pointer to the App's stack address

    __DSB(); // Ensure the VTOR and SP operations are complete
    __ISB(); // Flush the pipeline because of SP change

    // Re-enable all interrupts before new application running.
    __enable_irq();

    // Run the Application Reset Handler
    uint32_t reset = *(uint32_t *)(entry_point + 4);
    ((void (*)(void))(reset | 1U))();
    while (1); // No Warning.
}
