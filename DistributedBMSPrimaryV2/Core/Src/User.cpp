/*
 * FDCAN2: daughter CAN (IDs 0x100..). FDCAN3: vehicle CAN (0x040.., 0x1A0 commands).
 * I2C2: ADS1115 (pack current sense), INA226 (aux current).
 * SPI1 + NCS_A: ADS131M02 (24-bit ADC) bring-up; see g_ads131m02_* in debugger (aux task writes only).
 * IN0/IN1 + BTS71040 (SPI1, NCS_L): main contactor drive on PrimaryV2.
 * RX uses HAL_FDCAN_RxFifo0Callback in CanBus.cpp (multi-instance dispatch).
 */
#include "User.hpp"

#include "ads131m02.hpp"
#include "ads1115.hpp"
#include "BmsCanInterface.hpp"
#include "BmsFleet.hpp"
#include "BmsManager.hpp"
#include "PrimaryV2Contract.hpp"
#include "CanBus.hpp"
#include "cmsis_os.h"
#include "ina226.hpp"
#include "bts71040.hpp"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim8;

static Bts71040::Pins makeBts71040Pins() {
    Bts71040::Pins pins{};
    pins.ncs_port = NCS_L_GPIO_Port;
    pins.ncs_pin = NCS_L_Pin;
    pins.in_port[0] = IN1_GPIO_Port;
    pins.in_pin[0] = IN1_Pin;
    pins.in_port[1] = IN2_GPIO_Port;
    pins.in_pin[1] = IN2_Pin;
    pins.in_port[2] = IN3_GPIO_Port;
    pins.in_pin[2] = IN3_Pin;
    pins.in_port[3] = IN0_GPIO_Port;
    pins.in_pin[3] = IN0_Pin;
    return pins;
}

static CanBus daughter_can(hfdcan2);
static CanBus vehicle_can(hfdcan3);
static BmsFleet fleet;

static ADS1115 adc(&hi2c2, ADS1115::Addr7::GND, ADS1115::Pga::FS_4_096V, ADS1115::DataRate::SPS_128);
static INA226 ina(&hi2c2, INA226::I2C_ADDR_DEVICE1);
static INA226 ina2(&hi2c2, INA226::I2C_ADDR_DEVICE2);
static Bts71040 bts71040(&hspi1, makeBts71040Pins());

static BmsManager bms_manager(&fleet, &adc, &ina);
static BmsCanInterface vehicle_iface(vehicle_can, bms_manager);

static Ads131m02 ads131m02(&hspi1, {NCS_A_GPIO_Port, NCS_A_Pin});

static Bts71040* s_bts71040 = nullptr;
static osMutexId_t s_spi1_mutex{nullptr};

/** Live Expressions: contactor bring-up + current sensor debug. */
volatile uint8_t g_contactors_closed = 0;
volatile float g_pack_adc_V = 0.0f;
volatile float g_pack_current_A = 0.0f;
volatile float g_aux_current_A = 0.0f;
volatile float g_aux_bus_V = 0.0f;
volatile float g_aux_shunt_mV = 0.0f;
volatile uint8_t g_pack_adc_read_ok = 0;
volatile uint8_t g_aux_ina_read_ok = 0;
volatile float g_fan_current_A = 0.0f;
volatile float g_fan_bus_V = 0.0f;
volatile float g_fan_shunt_mV = 0.0f;
volatile uint8_t g_fan_ina_read_ok = 0;
volatile uint8_t g_contactor_out_mask = 0;  /* bit0=IN0/OUT0, bit1=IN1/OUT1 */
volatile uint8_t g_bts71040_out_readback = 0;
/** Stagger: 0=boot delay, 1=IN0 only, 2=both closed. */
volatile uint8_t g_contactor_bringup_stage = 0;
volatile uint32_t g_contactor_boot_delay_loops = 0;
volatile uint32_t g_contactor_stage1_loops = 0;

static constexpr uint8_t kBtsOutBothMask = 0x03u;  /* OUT0 + OUT1 */
/** Car bench: contactors + fan only; no daughter telemetry required. */
static constexpr bool kCarTestActuatorsOnly = true;
static constexpr bool kManualContactorStagger = true;
static constexpr uint32_t kContactorBootDelayMs = 1000;
static constexpr uint32_t kContactorStaggerDelayMs = 500;
static constexpr uint8_t kCarTestFanDutyPercent = 50;
/** Slower safety loop on bench so contactor/measurement sequence is easy to follow in debugger. */
static constexpr uint32_t kCarTestSafetyLoopMs = 100u;
static constexpr uint32_t kSafetyLoopMs =
    kCarTestActuatorsOnly ? kCarTestSafetyLoopMs : PrimaryV2Contract::BMS_MANAGER_PERIOD_MS;
static constexpr uint32_t kContactorBootDelayLoops =
    (kContactorBootDelayMs + kSafetyLoopMs - 1u) / kSafetyLoopMs;
static constexpr uint32_t kContactorStaggerLoops =
    (kContactorStaggerDelayMs + kSafetyLoopMs - 1u) / kSafetyLoopMs;

/**
 * Live Expressions: set to 1 to halt after each safety-loop pass (BKPT).
 * Resume once → one loop (measurements + contactor step), then halt again.
 */
volatile uint8_t g_debug_step_enable = 0;

/** Last ADS131M02 sample (updated in setup + ~500 ms in StartDefaultTask if init ok). */
volatile uint8_t g_ads131m02_init_ok = 0;
volatile uint32_t g_ads131m02_status = 0;
volatile int32_t g_ads131m02_ch0 = 0;
volatile int32_t g_ads131m02_ch1 = 0;

/** Debugger: copy updated under fleet mutex after each processModules (coherent snapshot). */
static FleetData g_fleet_debug_snapshot{};
volatile const FleetData* g_fleet_debug = nullptr;

static osMutexId_t s_fleet_mutex{nullptr};
static osMutexId_t s_bms_mutex{nullptr};

static void lockSpi1() {
    if (s_spi1_mutex != nullptr) {
        (void)osMutexAcquire(s_spi1_mutex, osWaitForever);
    }
}

static void unlockSpi1() {
    if (s_spi1_mutex != nullptr) {
        (void)osMutexRelease(s_spi1_mutex);
    }
}

/** BTS71040 HWCR + OUT off; IN0/IN1 low until BmsManager closes contactors. */
static void bts71040HwInit() {
    if (s_bts71040 == nullptr) {
        return;
    }

    HAL_GPIO_WritePin(IN0_GPIO_Port, IN0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    s_bts71040->setChannel(4, false);  /* IN0 */
    s_bts71040->setChannel(1, false);  /* IN1 */

    lockSpi1();
    (void)s_bts71040->writeHWCR(0x01, nullptr);
    (void)s_bts71040->writeHWCR(0x00, nullptr);
    (void)s_bts71040->writeOUT(0x00, nullptr);
    unlockSpi1();
}

static uint8_t readContactorOutMask() {
    uint8_t mask = 0;
    if (HAL_GPIO_ReadPin(IN0_GPIO_Port, IN0_Pin) == GPIO_PIN_SET) {
        mask |= 0x01u;
    }
    if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == GPIO_PIN_SET) {
        mask |= 0x02u;
    }
    return mask;
}

/** Mirror IN0/IN1 GPIO onto BTS71040 IN/OUT (required on PrimaryV2). */
static void bts71040SyncContactors(uint8_t out_mask) {
    if (s_bts71040 == nullptr) {
        return;
    }

    s_bts71040->setChannel(4, (out_mask & 0x01u) != 0u);  /* IN0 / OUT0 */
    s_bts71040->setChannel(1, (out_mask & 0x02u) != 0u);  /* IN1 / OUT1 */

    lockSpi1();
    (void)s_bts71040->writeOUT(out_mask & kBtsOutBothMask, nullptr);
    uint8_t rb = 0;
    if (s_bts71040->readOUT(rb) == HAL_OK) {
        g_bts71040_out_readback = rb;
    }
    unlockSpi1();
}

/** Drive IN0/IN1 and mirror onto BTS71040 OUT register. */
static void driveContactorMask(uint8_t out_mask) {
    HAL_GPIO_WritePin(IN0_GPIO_Port, IN0_Pin,
                      (out_mask & 0x01u) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin,
                      (out_mask & 0x02u) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET);
    bts71040SyncContactors(out_mask);
    g_contactor_out_mask = out_mask;
}

/**
 * Car-test contactor sequence (loop-count based, debugger-safe):
 *   1 s all open → IN0 → 500 ms → IN1 (both closed).
 */
static void maintainContactorStagger() {
    static uint32_t boot_delay_loops = 0;
    static uint32_t stage1_loops = 0;

    if (g_contactor_bringup_stage >= 2) {
        driveContactorMask(kBtsOutBothMask);
        g_contactors_closed = 1;
        return;
    }

    if (g_contactor_bringup_stage == 0) {
        driveContactorMask(0x00u);
        g_contactors_closed = 0;
        boot_delay_loops++;
        g_contactor_boot_delay_loops = boot_delay_loops;
        if (boot_delay_loops >= kContactorBootDelayLoops) {
            g_contactor_bringup_stage = 1;
            stage1_loops = 0;
            g_contactor_stage1_loops = 0;
            driveContactorMask(0x01u);
        }
        return;
    }

    stage1_loops++;
    g_contactor_stage1_loops = stage1_loops;
    driveContactorMask(0x01u);
    if (stage1_loops >= kContactorStaggerLoops) {
        g_contactor_bringup_stage = 2;
        driveContactorMask(kBtsOutBothMask);
        g_contactors_closed = 1;
    }
}

static void publishMeasurementDebug() {
    g_pack_adc_V = bms_manager.getLastPackAdcVoltage_V();
    g_pack_current_A = bms_manager.getBatteryCurrent_A();
    g_aux_current_A = bms_manager.getAuxCurrent_A();
    g_aux_bus_V = bms_manager.getLastAuxBusVoltage_V();
    g_aux_shunt_mV = bms_manager.getLastAuxShuntVoltage_V() * 1000.0f;
    g_pack_adc_read_ok = bms_manager.wasLastPackAdcReadOk() ? 1u : 0u;
    g_aux_ina_read_ok = bms_manager.wasLastAuxReadOk() ? 1u : 0u;
}

/** U403 fan rail INA226 @ 0x44 (+12V_P → FAN_Power / R513). */
static void pollFanIna226(uint32_t now_ms) {
    static uint32_t last_ms = 0;
    if ((now_ms - last_ms) < PrimaryV2Contract::INA226_READ_PERIOD_MS) {
        return;
    }
    last_ms = now_ms;

    INA226::Measurement m{};
    if (ina2.readMeasurement(m) == HAL_OK) {
        g_fan_current_A = m.current_A;
        g_fan_bus_V = m.bus_V;
        g_fan_shunt_mV = m.shunt_V * 1000.0f;
        g_fan_ina_read_ok = 1u;
    } else {
        g_fan_ina_read_ok = 0u;
    }
}

static void ads131m02TestSample() {
    uint16_t st = 0;
    int32_t c0 = 0;
    int32_t c1 = 0;
    if (ads131m02.readConversion(st, c0, c1) != HAL_OK) {
        return;
    }
    g_ads131m02_status = st;
    g_ads131m02_ch0 = c0;
    g_ads131m02_ch1 = c1;
}

void UserInitRtosSync(void) {
    s_fleet_mutex = osMutexNew(nullptr);
    s_bms_mutex = osMutexNew(nullptr);
    s_spi1_mutex = osMutexNew(nullptr);
    bms_manager.setFleetAccessMutex(s_fleet_mutex);
}

void setup() {
    (void)daughter_can.configureFilterAcceptAll();
    (void)daughter_can.start();
    (void)vehicle_can.configureFilterAcceptAll();
    (void)vehicle_can.start();

    fleet.registerDaughter(0x100, 0);
    fleet.registerDaughter(0x101, 1);
    fleet.registerDaughter(0x102, 2);

    (void)adc.init();
    /* U103 aux path: R102 shunt on +12V_BUCK → +12V_BUCK_A (populate value is 20 mΩ on this board). */
    (void)ina.init(0.02f, 100.0f);
    (void)ina2.init(0.02f, 100.0f);

    s_bts71040 = &bts71040;
    bts71040HwInit();

    if (!kManualContactorStagger) {
        bms_manager.setContactorGpio(IN0_GPIO_Port, IN0_Pin);
        bms_manager.setSecondContactorGpio(IN1_GPIO_Port, IN1_Pin);
    }

    bms_manager.setFanPwmTimer(&htim8, TIM_CHANNEL_2);

    BmsManager::Config cfg;
    cfg.cell_overvoltage_mV = 4220;
    cfg.cell_undervoltage_mV = 2500;
    cfg.overtemp_C = 100.0f;
    cfg.overcurrent_A = 100.0f;
    cfg.aux_overcurrent_A = 50.0f;
    /* L-channel idle (U201 bias ≈ 1.65 V; measure g_pack_adc_V at 0 A and use that). */
    cfg.current_shunt_resistance_ohm = 0.001f;
    cfg.current_gain = 50.0f;  /* 50 mV/A at ADS1115 AIN0 → 20 A/V */
    cfg.current_offset_V = 1.68f;
    cfg.aux_current_offset_A = 0.007f;  /* ~7 mA INA226 buck-path zero (PSU vs g_aux_current_A). */
    cfg.data_stale_timeout_ms = PrimaryV2Contract::DAUGHTER_STALE_TIMEOUT_MS;
    cfg.ads1115_read_period_ms = PrimaryV2Contract::ADS1115_READ_PERIOD_MS;
    cfg.ina226_read_period_ms = PrimaryV2Contract::INA226_READ_PERIOD_MS;
    bms_manager.setConfig(cfg);

    bms_manager.setDebugMode(true, false, true);
    if (kManualContactorStagger) {
        bms_manager.setExternalContactorControl(true);
    }

    bms_manager.init();
    if (kCarTestActuatorsOnly) {
        bms_manager.setFanPwmDuty(kCarTestFanDutyPercent);
    }

    BmsCanInterface::Config vcan_cfg{};
    vcan_cfg.heartbeat_period_ms = 100;
    vcan_cfg.pack_status_period_ms = 100;
    vehicle_iface.init(vcan_cfg);

    HAL_GPIO_WritePin(NCS_A_GPIO_Port, NCS_A_Pin, GPIO_PIN_SET);
    const HAL_StatusTypeDef ads_st = ads131m02.init();
    g_ads131m02_init_ok = (ads_st == HAL_OK) ? 1u : 0u;
    if (ads_st == HAL_OK) {
        ads131m02TestSample();
    }
}

void StartDefaultTask(void* argument) {
    (void)argument;
    for (;;) {
        CanBus::Frame rx{};
        (void)osMutexAcquire(s_fleet_mutex, osWaitForever);
        while (daughter_can.read(rx)) {
            fleet.handleMessage(rx, osKernelGetTickCount());
            HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
        }
        (void)osMutexRelease(s_fleet_mutex);
        // Keep RX task responsive; other work is split into dedicated tasks.
        osDelay(2);
    }
}

void StartFleetTask(void* argument) {
    (void)argument;
    for (;;) {
        const uint32_t now = osKernelGetTickCount();
        (void)now;
        (void)osMutexAcquire(s_fleet_mutex, osWaitForever);
        fleet.processModules();
        g_fleet_debug_snapshot = fleet.fleet();
        g_fleet_debug = &g_fleet_debug_snapshot;
        (void)osMutexRelease(s_fleet_mutex);
        osDelay(PrimaryV2Contract::FLEET_AGGREGATE_PERIOD_MS);
    }
}

void StartSafetyTask(void* argument) {
    (void)argument;
    for (;;) {
        const uint32_t now = osKernelGetTickCount();
        (void)osMutexAcquire(s_bms_mutex, osWaitForever);
        bms_manager.update(now);
        publishMeasurementDebug();
        (void)osMutexRelease(s_bms_mutex);

        if (kManualContactorStagger) {
            maintainContactorStagger();
            bms_manager.setReportedContactorsClosed(g_contactors_closed != 0u);
        } else {
            const uint8_t out_mask = readContactorOutMask();
            static uint8_t last_bts_out_mask = 0xFFu;
            if (out_mask != last_bts_out_mask) {
                bts71040SyncContactors(out_mask);
                last_bts_out_mask = out_mask;
            }
        }

        if (kCarTestActuatorsOnly) {
            bms_manager.setFanPwmDuty(kCarTestFanDutyPercent);
        }

        pollFanIna226(now);

        osDelay(kSafetyLoopMs);

        if (g_debug_step_enable != 0u) {
            __BKPT(0);
        }
    }
}

void StartVehicleTxTask(void* argument) {
    (void)argument;
    for (;;) {
        const uint32_t now = osKernelGetTickCount();
        (void)osMutexAcquire(s_bms_mutex, osWaitForever);
        vehicle_iface.update(now);
        (void)osMutexRelease(s_bms_mutex);
        osDelay(PrimaryV2Contract::VEHICLE_IFACE_PERIOD_MS);
    }
}

void StartAuxTask(void* argument) {
    (void)argument;
    for (;;) {
        if (g_ads131m02_init_ok != 0) {
            ads131m02TestSample();
        }
        osDelay(PrimaryV2Contract::ADS131M02_SAMPLE_PERIOD_MS);
    }
}
