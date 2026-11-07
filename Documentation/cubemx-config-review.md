# Hyades CubeMX Configuration Review

**File:** `Flight Controller V6/STM32IDE/Flight Controller V6.ioc` (saved 2026-10-22 11:43). Tools: CubeMX 6.15.0 and FW_H7 1.12.1, for the STM32H743VIT6 in LQFP100.

**Netlist:** MCU sch 2026-10-22 11:53.

**Reviewed:** 2026-10-22. This review lists open items only.

**Companion:** `claude/schematic-review-rev0.3.md`.

## Verdict

The configuration is now correct on paper:
- SPI3 runs at 12 MHz.
- The EXTI9_5, TIM2 and PVD interrupts are enabled.
- All four canards share TIM3 (PB4, PA7, PB0, PB1).
- All 39 signal pins match the netlist.

What's left is set-up, not errors: the ADC sequence, the TIM2 timebase, the memory layout and MPU, and generating code into one project. Today's .ioc sits alone, away from the project files.

## Open findings

| ID | Severity | Where | Finding | Fix | Basis |
|---|---|---|---|---|---|
| B1 | Medium | ADC1 | One rank (INP3) at 1.5 cycles from a 64 kΩ source; INP4 (VSERVO) isn't sequenced. On the H743, VREFINT is only on ADC3. | Two ranks at 810.5 cycles, DMA one-shot; calibrate at boot; read VREFINT on ADC3 to scale by the real VDDA. | [verified] .ioc |
| B2 | Medium | Linker / memory | The stack (0x400) and heap (0x200) are in AXI SRAM; DTCM, ITCM and D2 are unused; MPU and caches are off. | 8 KB stack in DTCM with a 32 B MPU guard; GNC state in DTCM; DMA buffers in non-cacheable D2; heap 0; then enable the caches. | [verified] .ld |
| B3 | Medium | Code generation | Three copies: the 11:43 .ioc alone in `Flight Controller V6/STM32IDE/`; the CubeIDE project (April code) with the 10:17 .ioc in `Hyades Rocket Files/STM32IDE/`; and the repo's July copy. CoupleFile is off. | Copy the 11:43 .ioc into the repo's `Firmware/STM32IDE/`; enable one .c/.h pair per peripheral; generate, build and commit; archive the other copies. | [verified] files |
| B4 | Medium | TIM2 | PPS capture only, with PSC 0. | PSC 239, ARR 0xFFFFFFFF: a free-running 1 MHz counter that wraps after 71.6 min. | [inference] |
| C1 | Low | TIM4 buzzer | 200 Hz frame; the buzzer type is undecided (schematic S4). | Active buzzer: 100 % duty or GPIO. Piezo: 2–4 kHz at 50 %. | [inference] |
| C2 | Low | SPI3/SPI4 | Data size, mode, NSS, Keep IO State and pin speed are at CubeMX defaults. | 8-bit, mode 0, hardware NSS off, Master Keep IO State on, pins at Medium speed. | [verify] |
| C3 | Low | NVIC | SPI4_IRQn (flash) is at priority 1, level with the IMU path. | Set it to 5. | [verified] .ioc |
| C4 | Low | PB4 | Resets as NJTRST with its pull-up on, so PWM1 sits high until TIM3 is initialised. | Initialise TIM3 early; check the servo ignores a held-high line, and add a pull-down if not. | [inference] |
| C5 | Low | BOR | Brown-out level is an option byte, not in the .ioc. | Set BOR_LEV to level 3 (≈2.7 V) in CubeProgrammer. | [verify] |

**Build-out (required by `h743_flight_architecture`):**
- DMA for SPI3, SPI4, ADC1 and UART RX, with IDLE detection on the UARTs.
- I-cache, D-cache and the MPU regions (B2).
- IWDG plus a windowed WWDG.
- The CRC unit.

## How-to notes

**B3, consolidate.** Close CubeMX first.
1. Copy the 11:43 .ioc over `Hyades-Flight-Computer/Firmware/STM32IDE/Flight Controller V6.ioc`, and open it from there.
2. In Project Manager → Code Generator, tick:
   - "Generate peripheral initialization as a pair of .c/.h files per peripheral"
   - "Keep User Code when re-generating"
3. Generate code, build, and commit.
4. Archive the other two folders.

**B4, TIM2.**
1. Clock Source: Internal Clock. Prescaler: 239. Counter Period: 4294967295. Auto-reload preload: Disable.
2. Input Capture CH1: Rising Edge, Direct, no prescaler, Input Filter ≈4.
3. In code:
   ```c
   HAL_TIM_Base_Start(&htim2);
   HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
   ```
4. Read time as `TIM2->CNT`, and take differences as `uint32_t` subtractions so they survive wrap.

**B1, ADC1.**
1. ADC1 settings:
   - Clock Prescaler: Asynchronous ÷1 (per_ck = HSE).
   - 16-bit resolution.
   - Scan mode on, continuous off.
   - EOC flag at end of sequence.
   - DMA One Shot.
   - Optional oversampling: ×16 with a right shift of 4.
   - 2 conversions, software trigger.
   - Rank 1: CH3; Rank 2: CH4; both at 810.5 cycles.
2. Add a DMA stream for ADC1: Normal, Half Word.
3. Add ADC3 with the Vrefint channel, polled.
4. At boot, calibrate both ADCs:
   ```c
   HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
   ```
   Repeat for `hadc3`.
5. Get VDDA from `__HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw, ADC_RESOLUTION_16B)`, in mV.

**B2, memory.**
1. Project Manager linker settings: heap 0x0, stack 0x2000.
2. Copy the generated .ld to a custom name, and point the linker setting at it so regeneration can't overwrite it.
3. Edit the custom .ld:
   - Remove the RAM_D1 `_estack` and `._user_heap_stack`.
   - Add a NOLOAD `.stack` section at the start of DTCMRAM, laid out as: a 32 B guard, then `_Min_Stack_Size`, with `_estack` at its end.
   - Add NOLOAD `.dtcm` (→ DTCMRAM) and `.dma` (32 B aligned → RAM_D2) sections.
4. Enable `__HAL_RCC_D2SRAM1/2/3_CLK_ENABLE()` at the top of `main` (the D2 SRAMs need their clocks on).
5. In CORTEX_M7:
   - I-cache and D-cache on.
   - MPU mode: "Background Region Privileged accesses only + MPU Disabled during hard fault, NMI and FAULTMASK handlers".
   - Region 0: 0x0, 4 GB, sub-region disable 0x87, no access, XN.
   - Region 1: 0x30000000, 32 KB, TEX 1, not cacheable, not bufferable, not shareable, full access, XN.
   - Region 2: 0x20000000, 32 B, no access, XN (the stack guard).
6. With heap 0, `malloc` (and newlib's `printf` of floats) fails by design.

**DMA, watchdogs and CRC.**
- **DMA1 streams:**

  | Peripheral | Mode | Width | Priority |
  |---|---|---|---|
  | SPI3 RX/TX | Normal | byte | Very High |
  | USART3/1 RX | Circular | byte | — |
  | SPI4 RX/TX | — | — | Low |
  | ADC1 | — | half-word | — |

  Stream IRQ priorities: SPI3 1, UART 4, SPI4 5, ADC 6.
- **IWDG1:**

  $$t=\frac{N\,(RL+1)}{f_\text{LSI}}$$

  where:
  - \(N\) is the prescaler (4–256).
  - \(RL\) is the reload value.
  - \(f_\text{LSI}\) ≈ 32 kHz.

  For example, \(N = 4\) and \(RL = 799\) give ≈100 ms.
- **WWDG1** (APB3 = 120 MHz):

  $$t=\frac{4096\cdot 2^{WDGTB}\,(T_{5:0}+1)}{f_\text{PCLK3}}$$

  where:
  - \(WDGTB\) selects the prescaler (1–128).
  - \(T_{5:0}\) is the counter's lower six bits.
  - \(f_\text{PCLK3}\) is the APB3 clock.

  With prescaler 8, counter 127 and window 111:
  - tick 273 µs;
  - timeout 17.5 ms;
  - earliest refresh at 4.4 ms.

  So kick it from the 100 Hz group. Enable the early-wakeup interrupt for logging.
- **Debug freeze:** `__HAL_DBGMCU_FREEZE_IWDG1()` and `__HAL_DBGMCU_FREEZE_WWDG1()`.
- **CRC:** default polynomial and init value; input inversion Byte and output inversion on, to match standard CRC-32.

## Clock summary

Matches the single-PLL plan:
- HSE 24 MHz → PLL1 960 MHz.
- P 480 MHz (CPU), Q 48 MHz (USB and SPI123), R 480 MHz (trace).
- AHB 240 MHz, APB 120 MHz.
- ADC from per_ck = HSE (24 → 12 MHz on rev V).
- SPI4 from HSE ÷8 = 3 MHz.
- SPI3 ÷4 = 12 MHz.
- PLL2, PLL3 and HSI48 unused.

The 960 MHz VCO is within the datasheet range; RM0433's 836 MHz figure is a confirmed typo. VOS0 is rated by AN5337 at T_j 105 °C (2 years at 100 % duty, 10 years at 20 %).

## Interrupts (.ioc → suggested)

| Interrupt | In .ioc | Suggested |
|---|---|---|
| PVD | 0 | 0 |
| EXTI9_5 | 1 | 1 |
| SPI3 | 1 | 1 |
| TIM2 | 3 | 3 |
| USART1/3 | 4 | 4 |
| SPI4 | 1 | **5** |
| OTG_FS | 6 | 6 |
| SysTick | 15 | 15 |

## Pins

All 39 match the netlist. Board-to-board pins, using MCU socket numbering:

| Pin | Signal | Socket pin |
|---|---|---|
| PB4 | PWM1 | U5.16 |
| PA7 | PWM2 | U5.15 |
| PB0 | PWM3 | U5.14 |
| PB1 | PWM4 | U5.13 |
| PE9 | PWM5 | U5.12 |
| PE11 | PWM6 | U5.11 |
| PA0 | PPS | U9.6 |
| PA6 | VBAT | U9.3 |
| PC4 | VSERVO | U9.1 |
| PD14 | Buzzer | U9.5 |
| PD8 | TX1 | U9.11 |
| PD9 | RX1 | U9.12 |

Free pins: PA5, PA15 and PE13/PE14. PB6–PB9 go to test points.

## Priority

1. B3: consolidate and generate.
2. B4: TIM2 timebase.
3. B1: ADC sequence and VREFINT.
4. B2: memory layout and MPU, then caches.
5. DMA, watchdogs and CRC.
6. The small items: C3, C2, C4, C5 and C1.

## Sources

- ST Community, VCO discrepancy: https://community.st.com/stm32-mcus-products-25/vco-frequency-discrepancy-in-stm32h743-datasheet-and-reference-manuals-36808
- AN5337: https://st.com/resource/en/application_note/an5337-guidelines-for-estimating-stm32h7-mcus-lifetime-stmicroelectronics.pdf
- ChibiOS forum, VREFINT on ADC3: https://forum.chibios.org/viewtopic.php?p=38254
- ST Community, enabling the D2 SRAMs: https://community.st.com/stm32-mcus-products-25/how-to-properly-enable-all-rams-on-a-single-core-stm32h7-61133
- ST Community, rev V ADC ÷2: https://community.st.com/stm32-mcus-products-25/adc-only-half-of-the-programmed-sample-rate-specs-changed-59872
- Linux `pinctrl-stm32h743.c`: https://android.googlesource.com/kernel/common/+/3f5f2283d6843bde6658e93fc50f5f00e5bd3595/drivers/pinctrl/stm32/pinctrl-stm32h743.c
- STM32CubeH7 `stm32h7xx_ll_utils.c`: https://pigweed.googlesource.com/third_party/github/STMicroelectronics/stm32h7xx_hal_driver/+/d5fc8d05fc16fa2a2a2f948cf6c6ab39e78358e1/Src/stm32h7xx_ll_utils.c
