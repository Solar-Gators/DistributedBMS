/*
 * FDCAN2: daughter CAN (IDs 0x100..). FDCAN3: vehicle CAN (0x040.., 0x1A0 commands).
 * I2C2: ADS1115 (pack current), INA226 @0x40 (aux), INA226 @0x44 (fan).
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

/** Vehicle integration without daughter boards: actuators + CAN only. */
static constexpr bool kVehicleIntegrationTest = true;
static constexpr bool kManualContactorStagger = true;
static constexpr uint32_t kContactorBootDelayMs = 1000;
static constexpr uint32_t kContactorStaggerDelayMs = 500;
static constexpr uint8_t kIntegrationTestFanDutyPercent = 50;

static osMutexId_t s_fleet_mutex{nullptr};
static osMutexId_t s_bms_mutex{nullptr};

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
    fleet.registerDaughter(0x103, 3);
    fleet.registerDaughter(0x104, 4);
    fleet.registerDaughter(0x105, 5);
    //fleet.registerDaughter(0x101, 1);

    (void)adc.init();
    /* U103 aux: R102 20 mΩ on +12V_BUCK → +12V_BUCK_A. U403 fan: R513 on +12V_P → FAN_Power. */
    (void)ina_aux.init(0.02f, 100.0f);
    (void)ina_fan.init(0.02f, 100.0f);

    s_bts71040 = &bts71040;
    bts71040HwInit();

    if (!kManualContactorStagger) {
        bms_manager.setContactorGpio(IN0_GPIO_Port, IN0_Pin);
        bms_manager.setSecondContactorGpio(IN1_GPIO_Port, IN1_Pin);
    }

    bms_manager.setFanPwmTimer(&htim8, TIM_CHANNEL_2);

    BmsManager::Config cfg;
    cfg.cell_overvoltage_mV = 4150;
    cfg.cell_undervoltage_mV = 2550;
    cfg.overtemp_C = 57.0f;
    cfg.discharge_overcurrent_A = 58.0f;
    cfg.charge_overcurrent_A = 24.0f;
    cfg.current_shunt_resistance_ohm = 0.001f;
    cfg.current_gain = 50.0f;  /* 50 mV/A at ADS1115 AIN0 → 20 A/V */
    cfg.current_offset_V = 1.68f;
    cfg.aux_current_offset_A = 0.007f;
    cfg.data_stale_timeout_ms = PrimaryV2Contract::DAUGHTER_STALE_TIMEOUT_MS;
    cfg.ads1115_read_period_ms = PrimaryV2Contract::ADS1115_READ_PERIOD_MS;
    cfg.ina226_read_period_ms = PrimaryV2Contract::INA226_READ_PERIOD_MS;
    bms_manager.setConfig(cfg);

    // Debug mode (WARNING: disables safety — testing only). Call after setConfig.
    // setDebugMode(true, true, true): force contactors + disable sensor faults.
    bms_manager.setDebugMode(false, false, false);

    bms_manager.init();
    if (kVehicleIntegrationTest) {
        bms_manager.setFanPwmDuty(kIntegrationTestFanDutyPercent);
    }

    BmsCanInterface::Config vcan_cfg{};
    vcan_cfg.module_count = 6;  // IDs 0x044..0x049 (matches registerDaughter slots)
    vcan_cfg.enable_fault_messages = false;
    vcan_cfg.enable_state_change_messages = false;
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
        CanBus::Frame rx{};
        (void)osMutexAcquire(s_fleet_mutex, osWaitForever);
        while (daughter_can.read(rx)) {

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

        if (kVehicleIntegrationTest) {
            bms_manager.setFanPwmDuty(kIntegrationTestFanDutyPercent);
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
