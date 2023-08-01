/*

Name:	Model4916_pwrtest.ino

Function:
        Test bench for power testing.

Copyright Notice:
        See accompanying LICENSE file.

Author:
        Pranau R, MCCI Corporation	July 2023

*/

#include <Catena.h>

#include <Arduino.h>
#include <Wire.h>
#include <arduino_lmic.h>
#include <Catena_Timer.h>
#include <Model4916.h>
#include <Model4916_cPCA9574.h>
#include <Model4916_c4916Gpios.h>
#include <Catena_CommandStream.h>
#include <Catena_Led.h>
#include <Catena_Log.h>
#include <Catena_Mx25v8035f.h>
#include <mcciadk_baselib.h>
#include <Catena_PollableInterface.h>

#include <SPI.h>

// load the fixup.
#include <Catena_CommandStream_vmicro_fixup.h>

using namespace McciCatena;
using namespace McciModel4916;

/****************************************************************************\
|
|       Manifest constants & typedefs.
|
\****************************************************************************/

constexpr uint8_t kBoosterEnable = D10;
constexpr uint8_t kBoosterPowerOn = D12;

static inline void BoosterEnable(void)
    {
    pinMode(kBoosterEnable, OUTPUT);
    digitalWrite(kBoosterEnable, HIGH);
    }

static inline void BoosterDisable(void)
    {
    pinMode(kBoosterEnable, INPUT);
    digitalWrite(kBoosterEnable, LOW);
    }

static inline void BoosterPowerOff(void)
    {
    pinMode(kBoosterPowerOn, OUTPUT);
    digitalWrite(kBoosterPowerOn, HIGH);
    }

static inline void BoosterPowerOn(void)
    {
    pinMode(kBoosterPowerOn, INPUT);
    digitalWrite(kBoosterPowerOn, LOW);
    }

/****************************************************************************\
|
|       Read-only data.
|
\****************************************************************************/

static const char sVersion[] = "1.0.0";

/****************************************************************************\
|
|       Variables.
|
\****************************************************************************/

// the Catena instance
Catena gCatena;

// the global I2C GPIO object for enabling power
cPCA9574                i2cgpiopower    { &Wire, 0 };
c4916Gpios gpiopower    { &i2cgpiopower };

// the global I2C GPIO object for enabling communication
cPCA9574                i2cgpioenable    { &Wire, 1 };
c4916Gpios gpioenable   { &i2cgpioenable };

McciCatena::cFram8k gFram;

//
// the LoRaWAN backhaul.  Note that we use the
// Catena version so it can provide hardware-specific
// information to the base class.
//
Catena::LoRaWAN gLoRaWAN;

// the LED instance object
StatusLed gLed (Catena::PIN_STATUS_LED);

// the second SPI bus, for use by flash
SPIClass gSPI2(
        Catena::PIN_SPI2_MOSI,
        Catena::PIN_SPI2_MISO,
        Catena::PIN_SPI2_SCK
        );

// The flash
Catena_Mx25v8035f gFlash;
bool gfFlash;

/****************************************************************************\
|
|       The command table
|
\****************************************************************************/

cCommandStream::CommandFn cmdReg, cmdSleep, cmdStandby, cmdStop, cmdWrite;

static const cCommandStream::cEntry sApplicationCommmands[] =
    {
    { "r", cmdReg },
    { "sleep", cmdSleep },
    { "standby", cmdStandby },
    { "stop", cmdStop },
    { "w", cmdWrite },
    // other commands go here....
    };

/* create the top level structure for the command dispatch table */
static cCommandStream::cDispatch
sApplicationCommandDispatch(
    sApplicationCommmands,          /* this is the pointer to the table */
    sizeof(sApplicationCommmands),  /* this is the size of the table */
    nullptr                         /* this is the "first word" for all the commands in this table*/
    );

/*

Name:	setup()

Function:
        Arduino setup function.

Definition:
        void setup(
            void
            );

Description:
        This function is called by the Arduino framework after
        basic framework has been initialized. We initialize the sensors
        that are present on the platform, set up the LoRaWAN connection,
        and (ultimately) return to the framework, which then calls loop()
        forever.

Returns:
        No explicit result.

*/

void setup(void)
    {
    Serial.begin(115200);
    Wire.begin();
    setup_gpio();
    setup_platform();

    // set up flash
    setup_flash();
    setup_commands();
    }

void setup_gpio()
    {
    if (! gpiopower.begin())
        Serial.println("GPIO to power-up modules failed to initialize");

    delay(100);

    if (! gpioenable.begin())
        Serial.println("GPIO to enable I2C interface failed to initialize");
    }

// set up the platform, print hello, etc.
void setup_platform()
    {
    // power-up and enable FRAM
    gpiopower.configVdd1(true);
    gpioenable.configVdd(true);
    gpiopower.setVdd1(true);
    delay(2);
    gpioenable.enableVdd1(true);
    delay(2);

    gCatena.begin();
    gLoRaWAN.begin(&gCatena);

    // power-up Flash
    gpiopower.configVSpi(true);
    delay(2);
    gpiopower.setVSpi(true);

    while (!Serial)
        /* wait for Serial to begin */
        yield();

    gCatena.SafePrintf("\n");
    gCatena.SafePrintf("-------------------------------------------------------------------------------\n");
    gCatena.SafePrintf("This is the Model4916_pwrtest program V%s.\n", sVersion);
    gCatena.SafePrintf("The program will now idle waiting for you to enter commands\n");
    gCatena.SafePrintf("Enter 'help' for a list of commands.\n");
    gCatena.SafePrintf("(remember to select 'Line Ending: Newline' at the bottom of the monitor window.)\n");

#ifdef CATENA_CFG_SYSCLK
    gCatena.SafePrintf("SYSCLK: %d MHz\n", CATENA_CFG_SYSCLK);
#endif

#ifdef USBCON
    gCatena.SafePrintf("USB enabled\n");
#else
    gCatena.SafePrintf("USB disabled\n");
#endif

    Catena::UniqueID_string_t CpuIDstring;

    gCatena.SafePrintf(
        "CPU Unique ID: %s\n",
        gCatena.GetUniqueIDstring(&CpuIDstring)
        );

    gCatena.SafePrintf("--------------------------------------------------------------------------------\n");
    gCatena.SafePrintf("\n");

    // set up the LED
    gLed.begin();
    gCatena.registerObject(&gLed);
    gLed.Set(LedPattern::OneThirtySecond);
    }

void setup_commands()
    {
    /* add our application-specific commands */
    gCatena.addCommands(
        /* name of app dispatch table, passed by reference */
        sApplicationCommandDispatch,
        /*
        || optionally a context pointer using static_cast<void *>().
        || normally only libraries (needing to be reentrant) need
        || to use the context pointer.
        */
        nullptr
        );
    }

void setup_flash(void)
    {
    // gSPI2.begin();
    if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
        {
        gfFlash = true;
        gFlash.powerDown();
        gCatena.SafePrintf("FLASH found, put power down\n");
        }
    else
        {
        gfFlash = false;
        gFlash.end();
        gSPI2.end();
        gCatena.SafePrintf("No FLASH found: check hardware\n");
        }

    gpiopower.setVSpi(false);
    }

/*

Name:	loop()

Function:
        Arduino polling function.

Definition:
        void loop(
            void
            );

Description:
        This function is called by the Arduino framework after
        initialization and setup() are complete. When it returns,
        the framework simply calls it again.

Returns:
        No explicit result.

*/

void loop()
    {
    // this drives the command processor, which in turn calls the command completion routines.
    gCatena.poll();
    }

/****************************************************************************\
|
|	The command functions
|
\****************************************************************************/

// argv[0] is "r"
// argv[1] is address to read
// argv[2] if present is the number of words

/* process "r base [len]" -- read and display len words of memory starting at base */
cCommandStream::CommandStatus cmdReg(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    uint32_t uLength;
    uint32_t uBase;
    cCommandStream::CommandStatus status;

    if (! (2 <= argc && argc <= 3))
        return cCommandStream::CommandStatus::kInvalidParameter;

    // get arg 2 as length; default is 32 bytes
    status = cCommandStream::getuint32(argc, argv, 2, 16, uLength, 1);

    if (status != cCommandStream::CommandStatus::kSuccess)
        return status;

    // get arg 1 as base; default is irrelevant
    status = cCommandStream::getuint32(argc, argv, 1, 16, uBase, 0);

    if (status != cCommandStream::CommandStatus::kSuccess)
        return status;

    if (uBase % 4 != 0)
        return cCommandStream::CommandStatus::kInvalidParameter;

    // dump the registers
    uint32_t buffer[8];
    for (uint32_t here = 0; here < uLength; here += sizeof(buffer))
        {
        char line[80];
        size_t n;

        n = uLength - here;
        if (n > sizeof(buffer)/sizeof(buffer[0]))
            n = sizeof(buffer)/sizeof(buffer[0]);

        std::memset(buffer, 0, n * sizeof(buffer[0]));

        /* once: */
            {
            auto p = reinterpret_cast<const volatile uint32_t *>(uBase + here);
            for (auto i = 0u; i < n; ++p, ++i)
                buffer[i] = *p;
            }

        unsigned iLine;

        iLine = McciAdkLib_Snprintf(line, sizeof(line), 0, "%08x:", uBase + here);
        for (auto i = 0u; i < n; ++i)
            {
            iLine += McciAdkLib_Snprintf(line, sizeof(line), iLine, " %08x", buffer[i]);
            }
        pThis->printf("%s\n", line);
        }

    return status;
    }

// argv[0] is "w"
// argv[1] is address to write
// argv[2..n-1] are values to write

/* process "w base v1 [v2 ...]" -- write words of memory starting at base */
cCommandStream::CommandStatus cmdWrite(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    uint32_t uBase;
    cCommandStream::CommandStatus status;

    if (argc < 3)
        return cCommandStream::CommandStatus::kInvalidParameter;

    // get arg 1 as base; default is irrelevant
    status = cCommandStream::getuint32(argc, argv, 1, 16, uBase, 0);

    if (status != cCommandStream::CommandStatus::kSuccess)
        return status;

    if (uBase % 4 != 0)
        return cCommandStream::CommandStatus::kInvalidParameter;

    // scan all the write paramters, and fail if any is bad
    for (int iArg = 2; iArg < argc; ++iArg)
        {
        uint32_t dummy;

        // get next arg; default is irrelevant
        status = cCommandStream::getuint32(argc, argv, iArg, 16, dummy, 0);

        if (status != cCommandStream::CommandStatus::kSuccess)
            return status;
        }

    // disable interrupts, saving state.
    uint32_t const flags = __get_PRIMASK();
    __disable_irq();

    // write values
    auto p = reinterpret_cast<volatile uint32_t *>(uBase);

    for (int iArg = 2; iArg < argc; ++iArg)
        {
        uint32_t value;

        // get next arg; default is irrelevant
        status = cCommandStream::getuint32(argc, argv, iArg, 16, value, 0);

        // can't happen, but it's ok to be safe.
        if (status != cCommandStream::CommandStatus::kSuccess)
            break;

        *p++ = value;
        }

    // restore interrupt state (if was enabled)
    __set_PRIMASK(flags);

    // return status of last fetch
    return status;
    }

// argv[0] is "sleep"
// argv[1] is the sleep delay
/* process "sleep"  */
cCommandStream::CommandStatus cmdSleep(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    if (argc > 2)
        return cCommandStream::CommandStatus::kInvalidParameter;

    // get arg 1 as sleep interval, default is 5 seconds.
    cCommandStream::CommandStatus status;
    uint32_t sleepInterval;
    status = cCommandStream::getuint32(argc, argv, 1, /* base */ 0, sleepInterval, 5);

    if (status != cCommandStream::CommandStatus::kSuccess)
        return status;

    pThis->printf("%s for %u seconds\n", argv[0], sleepInterval);
    delay(2000);

    BoosterPowerOff();
    gpioenable.enableVdd1(false);
    gpiopower.setVdd1(false);

    LedPattern const save_led = gLed.Set(LedPattern::Off);
    Serial.end();
    Wire.end();
    SPI.end();
    if (gfFlash)
        gSPI2.end();

    gCatena.Sleep(sleepInterval);

    Serial.begin();
    Wire.begin();
    SPI.begin();
    // if (gfFlash)
    // 	gSPI2.begin();

    // power-up FRAM
    gpiopower.configVdd1(true);
    gpioenable.configVdd(true);
    gpiopower.setVdd1(true);
    delay(2);
    gpioenable.enableVdd1(true);
    delay(2);
    BoosterPowerOn();

    gLed.Set(save_led);
    pThis->printf("awake again\n");

    // start the FRAM
    if (! gFram.begin())
        {
        gLog.printf(
            gLog.kError,
            "CatenaStm32L0::begin: FRAM begin() failed\n"
            );
        }

    // check whether the FRAM is valid
    if (! gFram.isValid())
        {
        gLog.printf(
            gLog.kError,
            "CatenaStm32L0::begin: FRAM contents are not valid, resetting\n"
            );
        gFram.initialize();
        }

    return status;
    }

// argv[0] is "standby"
/* process "standby"  */
cCommandStream::CommandStatus cmdStandby(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    if (argc > 1)
            return cCommandStream::CommandStatus::kInvalidParameter;

    pThis->printf("%s not implemented yet\n", argv[0]);
    return cCommandStream::CommandStatus::kSuccess;
    }

// argv[0] is "stop"
/* process "stop"  */
cCommandStream::CommandStatus cmdStop(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    if (argc > 1)
            return cCommandStream::CommandStatus::kInvalidParameter;

    pThis->printf("%s not implemented yet\n", argv[0]);
    return cCommandStream::CommandStatus::kSuccess;
    }