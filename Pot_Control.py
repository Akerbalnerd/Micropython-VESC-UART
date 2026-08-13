from machine import ADC
import time

from vesc_uart import VESC_UART


# ==================================================
# VESC CONFIGURATION
# ==================================================

USE_VESC_DEFAULTS = True


vesc = VESC_UART(
    use_defaults=USE_VESC_DEFAULTS,

    # These are only used when USE_VESC_DEFAULTS = False
    uart_id=0,
    baudrate=115200,
    tx_pin=0,
    rx_pin=1,

    min_duty=0.0,
    max_duty=0.25,

    debug=False,
)


# ==================================================
# POTENTIOMETER CONFIGURATION
# ==================================================

POT_PIN = 26

DEADBAND = 0.015

FILTER_STRENGTH = 0.15

LOOP_DELAY_MS = 20

PRINT_INTERVAL_MS = 250


pot = ADC(POT_PIN)


# ==================================================
# STARTUP
# ==================================================

print()
print("VESC Potentiometer Controller")
print("----------------------------")
print("Pot ADC pin:", POT_PIN)
print(
    "Max duty: {:.1f}%".format(
        vesc.max_duty * 100
    )
)
print()


filtered = 0.0

last_print = time.ticks_ms()


# ==================================================
# MAIN LOOP
# ==================================================

try:

    while True:

        raw = pot.read_u16()

        pot_position = raw / 65535.0


        # ------------------------------------------
        # Low-pass filter
        # ------------------------------------------

        filtered += (
            pot_position - filtered
        ) * FILTER_STRENGTH


        # ------------------------------------------
        # Deadband
        # ------------------------------------------

        if filtered <= DEADBAND:

            demand = 0.0

        else:

            demand = (
                filtered - DEADBAND
            ) / (
                1.0 - DEADBAND
            )


        # ------------------------------------------
        # Convert pot position to motor duty
        # ------------------------------------------

        duty = (
            vesc.min_duty
            + demand
            * (vesc.max_duty - vesc.min_duty)
        )


        # ------------------------------------------
        # Send command to VESC
        # ------------------------------------------

        vesc.set_duty(duty)


        # ------------------------------------------
        # Diagnostic shell output
        # ------------------------------------------

        now = time.ticks_ms()

        if time.ticks_diff(
            now,
            last_print
        ) >= PRINT_INTERVAL_MS:

            print(
                "ADC {:5d} | "
                "Pot {:5.1f}% | "
                "Duty {:5.1f}%".format(
                    raw,
                    filtered * 100,
                    duty * 100,
                )
            )

            last_print = now


        time.sleep_ms(LOOP_DELAY_MS)


except KeyboardInterrupt:

    vesc.stop()

    print()
    print("Controller stopped")