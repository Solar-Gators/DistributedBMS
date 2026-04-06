/*
 * FDCAN2: daughter CAN (IDs 0x100..). FDCAN3: vehicle CAN (0x040.., 0x1A0 commands).
 * I2C2: ADS1115 (pack current sense), INA226 (aux current).
 * IN2/IN3: main contactor drive lines (same roles as legacy Primary board).
 * RX uses HAL_FDCAN_RxFifo0Callback in CanBus.cpp (multi-instance dispatch).
 */
#include "User.hpp"

#include "ads1115.hpp"
#include "BmsCanInterface.hpp"
#include "BmsFleet.hpp"
#include "BmsManager.hpp"
#include "CanBus.hpp"
#include "cmsis_os.h"
#include "ina226.hpp"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern I2C_HandleTypeDef hi2c2;

static CanBus daughter_can(hfdcan2);
static CanBus vehicle_can(hfdcan3);
static BmsFleet fleet;

static ADS1115 adc(&hi2c2, ADS1115::Addr7::GND, ADS1115::Pga::FS_4_096V, ADS1115::DataRate::SPS_128);
static INA226 ina(&hi2c2, INA226::I2C_ADDR_BASE);

static BmsManager bms_manager(&fleet, &adc, &ina);
static BmsCanInterface vehicle_iface(vehicle_can, bms_manager);

/** Optional: raw aggregated struct; use bms_manager.getFleetSummary() for policy layer input. */
volatile const FleetData* g_fleet_debug = nullptr;

void setup() {
    (void)daughter_can.configureFilterAcceptAll();
    (void)daughter_can.start();
    (void)vehicle_can.configureFilterAcceptAll();
    (void)vehicle_can.start();

    fleet.registerDaughter(0x100, 0);
    //fleet.registerDaughter(0x101, 1);

    (void)adc.init();
    /* Shunt / I_max must match hardware (legacy Primary used 20 mΩ, 100 A max). */
    (void)ina.init(0.02f, 100.0f);

    bms_manager.setContactorGpio(IN2_GPIO_Port, IN2_Pin);
    bms_manager.setSecondContactorGpio(IN3_GPIO_Port, IN3_Pin);

    BmsManager::Config cfg;
    cfg.cell_overvoltage_mV = 4220;
    cfg.cell_undervoltage_mV = 2500;
    cfg.overtemp_C = 100.0f;
    cfg.overcurrent_A = 100.0f;
    cfg.aux_overcurrent_A = 50.0f;
    cfg.current_shunt_resistance_ohm = 0.001f;
    cfg.current_gain = 50.0f;
    cfg.current_offset_V = 0.0f;
    bms_manager.setConfig(cfg);

    bms_manager.init();

    BmsCanInterface::Config vcan_cfg{};
    vcan_cfg.heartbeat_period_ms = 100;
    vcan_cfg.pack_status_period_ms = 100;
    vehicle_iface.init(vcan_cfg);
}

void StartDefaultTask(void* argument) {
    (void)argument;
    uint32_t last_agg_ms = 0;

    for (;;) {
        CanBus::Frame rx{};
        while (daughter_can.read(rx)) {
            fleet.handleMessage(rx, osKernelGetTickCount());
            HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
        }

        const uint32_t now = osKernelGetTickCount();
        if ((now - last_agg_ms) >= 250u) {
            last_agg_ms = now;
            fleet.processModules();
            g_fleet_debug = &fleet.fleet();
        }

        /* BmsManager only closes contactors after requestContactorsClose(); legacy Primary
         * did this from the main loop in IDLE. Without this, state stays IDLE forever. */
        if (bms_manager.getState() == BmsManager::BmsState::IDLE &&
            bms_manager.getActiveFaults() == 0) {
            bms_manager.requestContactorsClose();
        }

        bms_manager.update(now);

        vehicle_iface.update(now);

        osDelay(10);
    }
}
