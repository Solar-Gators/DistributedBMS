/*
 * FDCAN2: daughter CAN (IDs 0x100..). FDCAN3: vehicle CAN (0x040.., 0x1A0 commands).
 * I2C2: ADS1115 (DHAB S/134 pack current on AIN0=low / AIN1=high), INA226 @0x40 (aux), INA226 @0x44 (fan).
 * SPI1 + NCS_A: ADS131M02 (24-bit ADC) bring-up.
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
static INA226 ina_aux(&hi2c2, INA226::I2C_ADDR_DEVICE1);
static INA226 ina_fan(&hi2c2, INA226::I2C_ADDR_DEVICE2);
static Bts71040 bts71040(&hspi1, makeBts71040Pins());

static BmsManager bms_manager(&fleet, &adc, &ina_aux, &ina_fan);
static BmsCanInterface vehicle_iface(vehicle_can, bms_manager);

static Ads131m02 ads131m02(&hspi1, {NCS_A_GPIO_Port, NCS_A_Pin});

static Bts71040* s_bts71040 = nullptr;
static osMutexId_t s_spi1_mutex{nullptr};

static constexpr uint8_t kBtsOutBothMask = 0x03u;  /* OUT0 + OUT1 */

/** Bench override for contactor/fan/fault bypass. Keep false for final behavior. */
static constexpr bool kQuickBenchTest = false;
/** false = production: fleet-driven faults, vehicle CAN close, temp-based fans. */
static constexpr bool kVehicleIntegrationTest = false;
/** false = BmsManager drives IN0/IN1 + stagger; true = boot auto-close in User.cpp. */
static constexpr bool kManualContactorStagger = false;
/**
 * Bench without vehicle CAN: auto-request contactor close once fleet is healthy.
 * Set false before vehicle integration (close comes from 0x1A0 on FDCAN3).
 */
static constexpr bool kBenchAutoCloseContactors = false;
static constexpr uint32_t kContactorBootDelayMs = 1000;
static constexpr uint32_t kContactorStaggerDelayMs = 500;
static constexpr uint8_t kIntegrationTestFanDutyPercent = 25;

static osMutexId_t s_fleet_mutex{nullptr};
static osMutexId_t s_bms_mutex{nullptr};
static osMutexId_t s_i2c2_mutex{nullptr};

volatile uint32_t g_dbg_can_rx_frames = 0;
volatile uint32_t g_dbg_fleet_cycle = 0;
volatile uint8_t g_dbg_daughter_online[MAX_MODULES]{};
volatile uint16_t g_dbg_daughter_avg_mV[MAX_MODULES]{};
volatile float g_dbg_daughter_avg_C[MAX_MODULES]{};
volatile float g_dbg_daughter_low_C[MAX_MODULES]{};
volatile uint16_t g_dbg_daughter_low_mV[MAX_MODULES]{};
volatile uint16_t g_dbg_daughter_high_mV[MAX_MODULES]{};
volatile uint16_t g_dbg_fleet_highest_mV = 0;
volatile uint16_t g_dbg_fleet_lowest_mV = 0;
volatile uint16_t g_dbg_fleet_imbalance_mV = 0;
volatile uint8_t g_dbg_fleet_highest_cell_idx = 0xFF;
volatile uint8_t g_dbg_fleet_lowest_cell_idx = 0xFF;
volatile uint32_t g_dbg_daughter_age_ms[MAX_MODULES]{};
volatile uint32_t g_dbg_daughter_last_update_ms[MAX_MODULES]{};
volatile uint16_t g_dbg_last_rx_id = 0;
volatile uint8_t g_dbg_last_rx_type = 0;
volatile int32_t g_dbg_last_rx_slot = -1;
volatile uint32_t g_dbg_rx_unknown_id = 0;
volatile uint32_t g_dbg_rx_decode_ok = 0;
volatile uint32_t g_dbg_rx_decode_fail = 0;
volatile uint32_t g_dbg_fdcan2_fifo_level = 0;
volatile uint32_t g_dbg_fdcan2_psr = 0;
volatile uint16_t g_dbg_active_faults = 0;
volatile uint8_t g_dbg_bms_state = 0;
volatile float g_dbg_pack_current_A = 0.0f;
volatile float g_dbg_pack_current_adc_V = 0.0f;
volatile float g_dbg_pack_current_adc_low_V = 0.0f;
volatile float g_dbg_pack_current_adc_high_V = 0.0f;
volatile float g_dbg_soc_percent = 0.0f;
volatile float g_dbg_aux_current_A = 0.0f;
volatile float g_dbg_fan_current_A = 0.0f;
volatile float g_dbg_aux_shunt_mV = 0.0f;
volatile float g_dbg_aux_bus_V = 0.0f;
volatile float g_dbg_fan_shunt_mV = 0.0f;
volatile float g_dbg_fan_bus_V = 0.0f;
volatile uint8_t g_dbg_ina_aux_init_ok = 0;
volatile uint16_t g_dbg_ina_aux_manuf_id = 0;
volatile uint32_t g_dbg_ina_aux_read_ok = 0;
volatile uint32_t g_dbg_ina_aux_read_fail = 0;
volatile uint32_t g_dbg_ina_aux_last_hal = 0;

static void refreshDaughterDebugMonitor()
{
    const auto& summary = fleet.summary();
    g_dbg_fleet_highest_mV = summary.highest_cell_mV;
    g_dbg_fleet_lowest_mV = summary.lowest_cell_mV;
    g_dbg_fleet_highest_cell_idx = summary.highest_cell_idx;
    g_dbg_fleet_lowest_cell_idx = summary.lowest_cell_idx;
    if (summary.highest_cell_mV >= summary.lowest_cell_mV) {
        g_dbg_fleet_imbalance_mV = summary.highest_cell_mV - summary.lowest_cell_mV;
    } else {
        g_dbg_fleet_imbalance_mV = 0;
    }

    for (uint8_t i = 0; i < MAX_MODULES; ++i)
    {
        const auto& m = fleet.moduleSnapshot(i);
        g_dbg_daughter_online[i] = m.valid ? 1u : 0u;
        g_dbg_daughter_avg_mV[i] = m.avg_cell_mV;
        g_dbg_daughter_avg_C[i] = m.avg_temp_C;
        g_dbg_daughter_low_C[i] = m.low_temp_C;
        g_dbg_daughter_low_mV[i] = m.low_mV;
        g_dbg_daughter_high_mV[i] = m.high_mV;
        g_dbg_daughter_age_ms[i] = m.age_ms;
        g_dbg_daughter_last_update_ms[i] = m.last_update_ms;
    }
    ++g_dbg_fleet_cycle;
}

static bool s_ads131m02_ready = false;

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

/** BTS71040 HWCR + OUT off; IN0/IN1 low until contactor sequence runs. */
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
    unlockSpi1();
}

/** Drive IN0/IN1 and mirror onto BTS71040 OUT register. */
static void driveContactorMask(uint8_t out_mask) {
    HAL_GPIO_WritePin(IN0_GPIO_Port, IN0_Pin,
                      (out_mask & 0x01u) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin,
                      (out_mask & 0x02u) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET);
    bts71040SyncContactors(out_mask);
}

/**
 * Contactor bring-up: 1 s all open → IN0 → 500 ms → both closed.
 * Returns whether both contactors are closed.
 */
static bool maintainContactorStagger(uint32_t now_ms) {
    static uint32_t boot_start_ms = 0;
    static uint32_t stage1_start_ms = 0;
    static uint8_t stage = 0;

    if (stage >= 2) {
        driveContactorMask(kBtsOutBothMask);
        return true;
    }

    if (boot_start_ms == 0) {
        boot_start_ms = now_ms;
    }

    if (stage == 0) {
        driveContactorMask(0x00u);
        if ((now_ms - boot_start_ms) >= kContactorBootDelayMs) {
            stage = 1;
            stage1_start_ms = now_ms;
            driveContactorMask(0x01u);
        }
        return false;
    }

    driveContactorMask(0x01u);
    if ((now_ms - stage1_start_ms) >= kContactorStaggerDelayMs) {
        stage = 2;
        driveContactorMask(kBtsOutBothMask);
        return true;
    }
    return false;
}

static void ads131m02PollSample() {
    uint16_t st = 0;
    int32_t c0 = 0;
    int32_t c1 = 0;
    (void)ads131m02.readConversion(st, c0, c1);
}

void UserInitRtosSync(void) {
    s_fleet_mutex = osMutexNew(nullptr);
    s_bms_mutex = osMutexNew(nullptr);
    s_i2c2_mutex = osMutexNew(nullptr);
    s_spi1_mutex = osMutexNew(nullptr);
    bms_manager.setFleetAccessMutex(s_fleet_mutex);
    bms_manager.setI2cMutex(s_i2c2_mutex);
}

void setup() {
    (void)daughter_can.configureFilterAcceptAll();
    (void)daughter_can.start();
    (void)vehicle_can.configureFilterAcceptAll();
    (void)vehicle_can.start();

    for (uint8_t i = 0; i < MAX_MODULES; ++i) {
        (void)fleet.registerDaughter(static_cast<uint16_t>(0x100u + i), i);
    }

    (void)adc.init();

    BmsManager::Config cfg;
    cfg.cell_overvoltage_mV = 4200;
    cfg.cell_undervoltage_mV = 2500;
    cfg.cell_imbalance_mV = 410;
    cfg.overtemp_C = 60.0f;
    cfg.overcurrent_A = 40.0f; /* FSGP charge OC (5 A×8P); abs trip */
    cfg.aux_overcurrent_A = 50.0f;
    cfg.enable_aux_overcurrent_fault = false;
    /* LEM DHAB S/134: AIN0=CH1 low (40 mV/A), AIN1=CH2 high (10 mV/A), 1 turn (race) */
    cfg.current_adc_channel_low = 0;
    cfg.current_adc_channel_high = 1;
    cfg.current_sensitivity_low_V_per_A = 0.040f;
    cfg.current_sensitivity_high_V_per_A = 0.010f;
    cfg.current_offset_V = 2.5f;
    cfg.current_sensor_turns = 15;
    cfg.current_range_crossover_A = 2.5f;
    cfg.current_adc_channel = 1;
    cfg.current_shunt_resistance_ohm = 1.0f;
    cfg.current_gain = 0.010f;
    cfg.aux_current_offset_A = 0.0f;
    cfg.data_stale_timeout_ms = PrimaryV2Contract::DAUGHTER_STALE_TIMEOUT_MS;
    cfg.ads1115_read_period_ms = PrimaryV2Contract::ADS1115_READ_PERIOD_MS;
    cfg.ina226_read_period_ms = PrimaryV2Contract::INA226_READ_PERIOD_MS;
    bms_manager.setConfig(cfg);

    /* U103 aux: R102 shunt on +12V_BUCK → +12V_BUCK_A. U403 fan: R513 on +12V_P. */
    const HAL_StatusTypeDef aux_init_st =
        ina_aux.init(cfg.aux_shunt_resistance_ohm, cfg.aux_max_current_A);
    const HAL_StatusTypeDef fan_init_st =
        ina_fan.init(cfg.fan_shunt_resistance_ohm, cfg.fan_max_current_A);

    uint16_t aux_manuf = 0;
    uint16_t fan_manuf = 0;
    const HAL_StatusTypeDef aux_probe_st = ina_aux.probe(aux_manuf);
    const HAL_StatusTypeDef fan_probe_st = ina_fan.probe(fan_manuf);
    g_dbg_ina_aux_init_ok =
        (aux_init_st == HAL_OK && aux_probe_st == HAL_OK && aux_manuf == 0x5449u) ? 1u : 0u;
    g_dbg_ina_aux_manuf_id = aux_manuf;

    if (aux_probe_st == HAL_OK) {
        INA226::Measurement m{};
        if (ina_aux.readMeasurement(m) == HAL_OK) {
            g_dbg_aux_shunt_mV = m.shunt_V * 1000.0f;
            g_dbg_aux_bus_V = m.bus_V;
            g_dbg_aux_current_A = m.current_A;
        }
    }
    if (fan_probe_st == HAL_OK) {
        INA226::Measurement m{};
        if (ina_fan.readMeasurement(m) == HAL_OK) {
            g_dbg_fan_shunt_mV = m.shunt_V * 1000.0f;
            g_dbg_fan_bus_V = m.bus_V;
            g_dbg_fan_current_A = m.current_A;
        }
    }
    {
        float v_low = 0.0f;
        float v_high = 0.0f;
        if (adc.readSingleEnded(cfg.current_adc_channel_low, v_low) == HAL_OK) {
            g_dbg_pack_current_adc_low_V = v_low;
        }
        if (adc.readSingleEnded(cfg.current_adc_channel_high, v_high) == HAL_OK) {
            g_dbg_pack_current_adc_high_V = v_high;
            g_dbg_pack_current_adc_V = v_high;
            const float turns = static_cast<float>(cfg.current_sensor_turns);
            const float denom = cfg.current_sensitivity_high_V_per_A * turns;
            if (denom > 0.0f) {
                g_dbg_pack_current_A = (v_high - cfg.current_offset_V) / denom;
            }
        }
    }
    (void)fan_init_st;
    (void)fan_probe_st;
    (void)fan_manuf;

    s_bts71040 = &bts71040;
    bts71040HwInit();

    bms_manager.setContactorGpio(IN0_GPIO_Port, IN0_Pin);
    bms_manager.setSecondContactorGpio(IN1_GPIO_Port, IN1_Pin);
    if (kManualContactorStagger) {
        bms_manager.setExternalContactorControl(true);
    }

    bms_manager.setFanPwmTimer(&htim8, TIM_CHANNEL_2);

    if (kVehicleIntegrationTest) {
        bms_manager.setDebugMode(true, false, true);
        bms_manager.setFanPwmDuty(kIntegrationTestFanDutyPercent);
    } else {
        bms_manager.setDebugMode(false);
    }

    bms_manager.init();

    BmsCanInterface::Config vcan_cfg{};
    vcan_cfg.heartbeat_period_ms = 100;
    vcan_cfg.pack_status_period_ms = 100;
    vcan_cfg.aux_current_period_ms = 100;
    vcan_cfg.fan_current_period_ms = 100;
    vehicle_iface.init(vcan_cfg);

    HAL_GPIO_WritePin(NCS_A_GPIO_Port, NCS_A_Pin, GPIO_PIN_SET);
    s_ads131m02_ready = (ads131m02.init() == HAL_OK);
    if (s_ads131m02_ready) {
        ads131m02PollSample();
    }
}

void StartDefaultTask(void* argument) {
    (void)argument;
    for (;;) {
        g_dbg_fdcan2_fifo_level =
            HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
        g_dbg_fdcan2_psr = hfdcan2.Instance->PSR;

        CanBus::Frame rx{};
        (void)osMutexAcquire(s_fleet_mutex, osWaitForever);
        while (daughter_can.read(rx)) {
            ++g_dbg_can_rx_frames;
            g_dbg_last_rx_id = static_cast<uint16_t>(rx.id & 0x7FFu);
            g_dbg_last_rx_type = (rx.len > 0u) ? rx.data[0] : 0xFFu;
            const int slot = fleet.slotForStdCanId(g_dbg_last_rx_id);
            g_dbg_last_rx_slot = slot;
            if (slot < 0) {
                ++g_dbg_rx_unknown_id;
            }
            fleet.handleMessage(rx, osKernelGetTickCount());
            HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
        }
        (void)osMutexRelease(s_fleet_mutex);
        osDelay(2);
    }
}

void StartFleetTask(void* argument) {
    (void)argument;
    for (;;) {
        (void)osMutexAcquire(s_fleet_mutex, osWaitForever);
        fleet.processModules();
        refreshDaughterDebugMonitor();
        g_dbg_rx_decode_ok = fleet.decodeOkCount();
        g_dbg_rx_decode_fail = fleet.decodeFailCount();
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

        if (kVehicleIntegrationTest) {
            bms_manager.setFanPwmDuty(kIntegrationTestFanDutyPercent);
        }

        (void)osMutexRelease(s_bms_mutex);

        g_dbg_active_faults = bms_manager.getActiveFaults();
        g_dbg_bms_state = static_cast<uint8_t>(bms_manager.getState());
        g_dbg_pack_current_A = bms_manager.getBatteryCurrent_A();
        g_dbg_pack_current_adc_V = bms_manager.getPackCurrentAdc_V();
        g_dbg_pack_current_adc_low_V = bms_manager.getPackCurrentAdcLow_V();
        g_dbg_pack_current_adc_high_V = bms_manager.getPackCurrentAdcHigh_V();
        g_dbg_soc_percent = bms_manager.getStateOfCharge();
        g_dbg_aux_current_A = bms_manager.getAuxCurrent_A();
        g_dbg_fan_current_A = bms_manager.getFanCurrent_A();
        g_dbg_aux_shunt_mV = bms_manager.getAuxShuntVoltage_V() * 1000.0f;
        g_dbg_aux_bus_V = bms_manager.getAuxBusVoltage_V();
        g_dbg_fan_shunt_mV = bms_manager.getFanShuntVoltage_V() * 1000.0f;
        g_dbg_fan_bus_V = bms_manager.getFanBusVoltage_V();
        g_dbg_ina_aux_read_ok = bms_manager.getAuxCurrentReadOkCount();
        g_dbg_ina_aux_read_fail = bms_manager.getAuxCurrentReadFailCount();
        g_dbg_ina_aux_last_hal = bms_manager.getAuxCurrentLastHalStatus();

        if (kManualContactorStagger) {
            const bool closed = maintainContactorStagger(now);
            bms_manager.setReportedContactorsClosed(closed);
        } else {
            const uint8_t out_mask = readContactorOutMask();
            static uint8_t last_bts_out_mask = 0xFFu;
            if (out_mask != last_bts_out_mask) {
                bts71040SyncContactors(out_mask);
                last_bts_out_mask = out_mask;
            }
        }

        osDelay(PrimaryV2Contract::BMS_MANAGER_PERIOD_MS);
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
        if (s_ads131m02_ready) {
            ads131m02PollSample();
        }
        osDelay(PrimaryV2Contract::ADS131M02_SAMPLE_PERIOD_MS);
    }
}
