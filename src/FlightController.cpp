#include "FlightController.h"

#include <cmath>

#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"


/////////////////////////////////////////////////////////////////
// Configuration
/////////////////////////////////////////////////////////////////

// Tower Pro MG90S on GPIO10 (PWM slice 5, channel A).
// 1000-2000 us is conservative (~90-100 deg of travel).  Most MG90S units
// reach full travel around 500-2400 us; widen only after checking that the
// servo does not buzz against its end stops.
static const uint     SERVO_PIN       = 10;
static const uint32_t SERVO_MIN_US    = 1'000;
static const uint32_t SERVO_MAX_US    = 2'000;
static const uint32_t SERVO_PERIOD_US = 20'000;  // 50 Hz frame

static const uint32_t TICK_US                 = 20'000;     // 50 Hz control loop
static const uint64_t STALL_TIMEOUT_US        = 5'000'000;  // restart Core 1 if heartbeat stops this long
static const uint64_t FLASH_PARK_TIMEOUT_US   = 50'000;
static const uint32_t FLASH_PARK_MAGIC        = 0x464C5348; // "FLSH"
static const float    DEFAULT_SERVO_ANGLE_DEG = 90.0f;

// Power-up self-test: slow sweep so wiring and power can be checked by eye.
static const bool     STARTUP_SWEEP_ENABLED   = true;


/////////////////////////////////////////////////////////////////
// Shared state
/////////////////////////////////////////////////////////////////

static critical_section_t lock_;

// guarded by lock_
static FcGpsSample gps_;
static FcStatus    status_;
static float       manualAngleDeg_ = DEFAULT_SERVO_ANGLE_DEG;

// lock-free, single writer each
static volatile uint32_t heartbeat_       = 0;      // Core 1 writes
static volatile bool     core1Ready_      = false;  // Core 1 sets once park IRQ installed, Core 0 clears on reset
static volatile bool     parkRequested_   = false;  // Core 0 writes
static volatile bool     parked_          = false;  // Core 1 writes
static volatile bool     relaunchNeeded_  = false;  // Core 0 only
static volatile uint32_t pwmClockHz_      = 0;      // clk_sys the PWM divider was last set for
static uint32_t          restartCount_    = 0;      // Core 0 only
static int               alarmNum_        = -1;

static uint servoSlice_ = 0;
static uint servoChan_  = 0;


/////////////////////////////////////////////////////////////////
// Control law
/////////////////////////////////////////////////////////////////

// Called on Core 1 every tick.  Replace the body with the flight control law.
//
// gps.seq == 0 means no fix has ever arrived.  gpsAgeMs is how old the latest
// fix is.  Expect fixes roughly every second while GPS is on between
// transmissions, and none at all while the radio is transmitting (up to
// ~10 min in a window, depending on how many slots transmit).
static float ComputeServoAngleDeg(const FcGpsSample &gps, uint32_t gpsAgeMs, float manualAngleDeg)
{
    (void)gps;
    (void)gpsAgeMs;

    return manualAngleDeg;
}


/////////////////////////////////////////////////////////////////
// Startup sweep
/////////////////////////////////////////////////////////////////

// Core 1 only.  Not reset on Core 1 restart, so the sweep runs once per boot.
static bool startupSweepDone_ = !STARTUP_SWEEP_ENABLED;

// 90 -> 60 -> 120 -> 90 at 20 deg/s, ~7 s total.  Slow on purpose, to keep
// current draw low.  Marks itself done after the last leg.
static float StartupSweepAngleDeg(uint32_t tick)
{
    struct Leg { float fromDeg; float toDeg; uint32_t ticks; };
    static const Leg LEG_LIST[] = {
        {  90.0f,  90.0f,  50 },  // hold 1 s
        {  90.0f,  60.0f,  75 },  // 1.5 s
        {  60.0f, 120.0f, 150 },  // 3 s
        { 120.0f,  90.0f,  75 },  // 1.5 s
    };

    for (const Leg &leg : LEG_LIST)
    {
        if (tick < leg.ticks)
        {
            return leg.fromDeg + (leg.toDeg - leg.fromDeg) * ((float)tick / (float)leg.ticks);
        }
        tick -= leg.ticks;
    }

    startupSweepDone_ = true;

    return DEFAULT_SERVO_ANGLE_DEG;
}


/////////////////////////////////////////////////////////////////
// Servo PWM
/////////////////////////////////////////////////////////////////

// 1 us per PWM count, so the channel level is the pulse width in us.
// Exact at 6 and 48 MHz; fractional divider covers other clocks.
static void ServoApplyClock()
{
    uint32_t hz    = clock_get_hz(clk_sys);
    uint32_t div16 = (uint32_t)(((uint64_t)hz * 16 + 500'000) / 1'000'000);

    pwm_set_clkdiv_int_frac(servoSlice_, div16 >> 4, div16 & 0xF);
    pwmClockHz_ = hz;
}

static void ServoSetup()
{
    servoSlice_ = pwm_gpio_to_slice_num(SERVO_PIN);
    servoChan_  = pwm_gpio_to_channel(SERVO_PIN);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, SERVO_PERIOD_US - 1);
    pwm_init(servoSlice_, &cfg, false);
    ServoApplyClock();
    pwm_set_chan_level(servoSlice_, servoChan_, 0);
    pwm_set_enabled(servoSlice_, true);

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
}

// Returns the angle actually applied.
static float ServoWriteAngle(float angleDeg)
{
    if (!std::isfinite(angleDeg)) { angleDeg = DEFAULT_SERVO_ANGLE_DEG; }
    if (angleDeg <   0.0f)        { angleDeg =   0.0f; }
    if (angleDeg > 180.0f)        { angleDeg = 180.0f; }

    uint32_t pulseUs = SERVO_MIN_US + (uint32_t)((angleDeg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US) + 0.5f);
    pwm_set_chan_level(servoSlice_, servoChan_, (uint16_t)pulseUs);

    return angleDeg;
}


/////////////////////////////////////////////////////////////////
// Flash safety (Core 1 side)
/////////////////////////////////////////////////////////////////

// While Core 0 erases/programs flash, XIP is unavailable, so Core 1 must not
// fetch code or data from flash.  Core 0 pushes a word into Core 1's FIFO;
// this RAM-resident handler then spins in RAM with interrupts off until
// released.
//
// The SDK's flash_safe_execute / multicore_lockout can't be used: with
// FreeRTOS linked it refuses to lock out the other core, and the lockout
// reply would land in Core 0's FIFO, which FreeRTOS drains.
static void __not_in_flash_func(Core1FifoIrq)()
{
    while (sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS) { (void)sio_hw->fifo_rd; }
    sio_hw->fifo_st = 0xFF;  // clear sticky error flags

    if (!parkRequested_) { return; }

    uint32_t save = save_and_disable_interrupts();

    parked_ = true;
    __sev();

    while (parkRequested_) { __wfe(); }

    parked_ = false;
    __sev();

    restore_interrupts(save);
}


/////////////////////////////////////////////////////////////////
// Core 1 main loop
/////////////////////////////////////////////////////////////////

static volatile bool tickDue_ = false;

static void OnTickAlarm(uint)
{
    tickDue_ = true;
}

static void Core1Main()
{
    // install flash-park handler before Core 0 may rely on it
    irq_set_exclusive_handler(SIO_IRQ_PROC1, Core1FifoIrq);
    while (sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS) { (void)sio_hw->fifo_rd; }
    sio_hw->fifo_st = 0xFF;
    irq_set_enabled(SIO_IRQ_PROC1, true);
    core1Ready_ = true;

    ServoSetup();

    // own hardware alarm with its IRQ on this core, so ticks don't depend on
    // Core 0 interrupts (unlike sleep_ms)
    hardware_alarm_set_callback(alarmNum_, OnTickAlarm);
    // set_callback only enables the NVIC line the first time the handler is
    // installed; a relaunched Core 1 has a fresh NVIC, so enable it here
    irq_set_enabled(TIMER_IRQ_0 + alarmNum_, true);

    absolute_time_t next = get_absolute_time();
    uint32_t sweepTick = 0;

    while (true)
    {
        next = delayed_by_us(next, TICK_US);
        tickDue_ = false;
        if (hardware_alarm_set_target(alarmNum_, next))
        {
            // fell behind, run now and resync
            tickDue_ = true;
            next = get_absolute_time();
        }

        // wfi with interrupts masked so a tick landing between the check and
        // the wfi still wakes us; handlers run once interrupts are restored
        uint32_t save = save_and_disable_interrupts();
        while (!tickDue_)
        {
            __wfi();
            restore_interrupts(save);
            save = save_and_disable_interrupts();
        }
        restore_interrupts(save);

        // backstop for clock changes Core 0 didn't report
        if (clock_get_hz(clk_sys) != pwmClockHz_)
        {
            ServoApplyClock();
        }

        critical_section_enter_blocking(&lock_);
        FcGpsSample gps    = gps_;
        float       manual = manualAngleDeg_;
        critical_section_exit(&lock_);

        uint32_t gpsAgeMs = UINT32_MAX;
        if (gps.seq != 0)
        {
            gpsAgeMs = (uint32_t)((time_us_64() - gps.rxTimeUs) / 1'000);
        }

        float angle = startupSweepDone_                  ?
                      ComputeServoAngleDeg(gps, gpsAgeMs, manual) :
                      StartupSweepAngleDeg(sweepTick++);
        float applied = ServoWriteAngle(angle);

        heartbeat_ = heartbeat_ + 1;

        critical_section_enter_blocking(&lock_);
        status_.ticks              = heartbeat_;
        status_.gpsSeqUsed         = gps.seq;
        status_.gpsAgeMs           = gpsAgeMs;
        status_.servoAngleCentiDeg = (int32_t)(applied * 100.0f);
        critical_section_exit(&lock_);
    }
}


/////////////////////////////////////////////////////////////////
// Core 0 API
/////////////////////////////////////////////////////////////////

void FcInit()
{
    critical_section_init(&lock_);
    alarmNum_ = hardware_alarm_claim_unused(true);
}

void FcLaunch()
{
    core1Ready_     = false;
    relaunchNeeded_ = false;

    multicore_reset_core1();

    // Core 1 may have been reset while parked or holding lock_.  Core 0 never
    // holds lock_ here, so any holder was Core 1; release it.
    parked_ = false;
    uint32_t save = save_and_disable_interrupts();
    spin_unlock_unsafe(lock_.spin_lock);
    restore_interrupts(save);

    multicore_launch_core1(Core1Main);
}

void FcPublishGps(const FcGpsSample &sample)
{
    critical_section_enter_blocking(&lock_);
    uint32_t seq = gps_.seq + 1;
    gps_          = sample;
    gps_.seq      = seq;
    gps_.rxTimeUs = time_us_64();
    critical_section_exit(&lock_);
}

bool FcSetServoAngle(float angleDeg)
{
    if (!std::isfinite(angleDeg)) { return false; }

    critical_section_enter_blocking(&lock_);
    manualAngleDeg_ = angleDeg;
    critical_section_exit(&lock_);

    return true;
}

FcStatus FcGetStatus()
{
    critical_section_enter_blocking(&lock_);
    FcStatus status = status_;
    critical_section_exit(&lock_);

    return status;
}

void FcOnClockChange()
{
    if (core1Ready_)
    {
        ServoApplyClock();
    }
}

void FcSupervise()
{
    static uint32_t lastHeartbeat    = 0;
    static uint64_t timeLastChangeUs = 0;

    uint64_t timeNowUs = time_us_64();
    uint32_t heartbeat = heartbeat_;

    if (relaunchNeeded_ == false && heartbeat != lastHeartbeat)
    {
        lastHeartbeat    = heartbeat;
        timeLastChangeUs = timeNowUs;
        return;
    }

    if (relaunchNeeded_ || timeNowUs - timeLastChangeUs > STALL_TIMEOUT_US)
    {
        ++restartCount_;
        FcLaunch();

        lastHeartbeat    = heartbeat_;
        timeLastChangeUs = timeNowUs;
    }
}

uint32_t FcGetRestartCount()
{
    return restartCount_;
}


/////////////////////////////////////////////////////////////////
// Flash safety (Core 0 side), called by picoinf around flash erase/program
/////////////////////////////////////////////////////////////////

extern "C" void PicoInfFlashOpBegin()
{
    if (core1Ready_ == false) { return; }

    parkRequested_ = true;
    __dmb();

    if (sio_hw->fifo_st & SIO_FIFO_ST_RDY_BITS)
    {
        sio_hw->fifo_wr = FLASH_PARK_MAGIC;
    }
    __sev();

    uint64_t deadline = time_us_64() + FLASH_PARK_TIMEOUT_US;
    while (parked_ == false)
    {
        if (time_us_64() > deadline)
        {
            // Core 1 is unresponsive.  Hold it in reset (bootrom, not flash)
            // so it can't fault during the flash op; supervisor relaunches it.
            multicore_reset_core1();
            core1Ready_     = false;
            relaunchNeeded_ = true;
            parkRequested_  = false;
            return;
        }
    }
}

extern "C" void PicoInfFlashOpEnd()
{
    if (parkRequested_ == false) { return; }

    parkRequested_ = false;
    __dmb();
    __sev();

    // wait for Core 1 to leave the park loop, so a back-to-back flash op
    // can't mistake the stale parked_ flag for a fresh acknowledgement
    uint64_t deadline = time_us_64() + FLASH_PARK_TIMEOUT_US;
    while (parked_ && time_us_64() <= deadline) {}
}
