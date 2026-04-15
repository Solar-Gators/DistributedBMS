/*
 * FDCAN2: daughter CAN (IDs 0x100..). FDCAN3: vehicle CAN (0x040.., 0x1A0 commands).
 * I2C2: ADS1115 (pack current sense), INA226 (aux current).
 * SPI1 + NCS_A: ADS131M02 (24-bit ADC) bring-up; see g_ads131m02_* in debugger (aux task writes only).
 * IN2/IN3: main contactor drive lines (same roles as legacy Primary board).
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
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim8;

static CanBus daughter_can(hfdcan2);
static CanBus vehicle_can(hfdcan3);
static BmsFleet fleet;

static ADS1115 adc(&hi2c2, ADS1115::Addr7::GND, ADS1115::Pga::FS_4_096V, ADS1115::DataRate::SPS_128);
static INA226 ina(&hi2c2, INA226::I2C_ADDR_BASE);

static BmsManager bms_manager(&fleet, &adc, &ina);
static BmsCanInterface vehicle_iface(vehicle_can, bms_manager);

static Ads131m02 ads131m02(&hspi1, {NCS_A_GPIO_Port, NCS_A_Pin});

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
    /* Shunt / I_max must match hardware (legacy Primary used 20 mΩ, 100 A max). */
    (void)ina.init(0.02f, 100.0f);

    bms_manager.setContactorGpio(IN2_GPIO_Port, IN2_Pin);
    bms_manager.setSecondContactorGpio(IN3_GPIO_Port, IN3_Pin);

    bms_manager.setFanPwmTimer(&htim8, TIM_CHANNEL_2);

    //bms_manager.setFanPwmDuty(50);

    BmsManager::Config cfg;
    cfg.cell_overvoltage_mV = 4220;
    cfg.cell_undervoltage_mV = 2500;
    cfg.overtemp_C = 100.0f;
    cfg.overcurrent_A = 100.0f;
    cfg.aux_overcurrent_A = 50.0f;
    cfg.current_shunt_resistance_ohm = 0.001f;
    cfg.current_gain = 50.0f;
    cfg.current_offset_V = 0.0f;
    cfg.data_stale_timeout_ms = PrimaryV2Contract::DAUGHTER_STALE_TIMEOUT_MS;
    cfg.ads1115_read_period_ms = PrimaryV2Contract::ADS1115_READ_PERIOD_MS;
    cfg.ina226_read_period_ms = PrimaryV2Contract::INA226_READ_PERIOD_MS;
    bms_manager.setConfig(cfg);

    bms_manager.init();

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
        (void)osMutexRelease(s_bms_mutex);
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
        if (g_ads131m02_init_ok != 0) {
            ads131m02TestSample();
        }
        osDelay(PrimaryV2Contract::ADS131M02_SAMPLE_PERIOD_MS);
    }
}
