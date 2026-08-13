from machine import UART, Pin
import time


class VESC_UART:

    # ==========================================================
    # VESC COMMAND IDs
    # ==========================================================

    COMM_FW_VERSION = 0
    COMM_GET_VALUES = 4

    COMM_SET_DUTY = 5
    COMM_SET_CURRENT = 6
    COMM_SET_CURRENT_BRAKE = 7
    COMM_SET_RPM = 8
    COMM_SET_POS = 9


    # ==========================================================
    # DEFAULT CONFIGURATION
    # ==========================================================

    DEFAULT_CONFIG = {

        # UART
        "uart_id": 0,
        "baudrate": 115200,
        "tx_pin": 0,
        "rx_pin": 1,

        # Communication
        "request_timeout_ms": 100,

        # Safety limits
        "min_duty": 0.0,
        "max_duty": 0.25,

        "max_current": 10.0,
        "max_brake_current": 10.0,

        "max_rpm": 10000,

        # Base library diagnostics
        "debug": False,
    }


    # ==========================================================
    # FAULT CODES
    # ==========================================================

    FAULT_CODES = {
        0: "NONE",
        1: "OVER_VOLTAGE",
        2: "UNDER_VOLTAGE",
        3: "DRV",
        4: "ABS_OVER_CURRENT",
        5: "OVER_TEMP_FET",
        6: "OVER_TEMP_MOTOR",
        7: "GATE_DRIVER_OVER_VOLTAGE",
        8: "GATE_DRIVER_UNDER_VOLTAGE",
        9: "MCU_UNDER_VOLTAGE",
        10: "WATCHDOG_RESET",
        11: "ENCODER_SPI",
        12: "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
        13: "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
        14: "FLASH_CORRUPTION",
        15: "HIGH_OFFSET_CURRENT_SENSOR_1",
        16: "HIGH_OFFSET_CURRENT_SENSOR_2",
        17: "HIGH_OFFSET_CURRENT_SENSOR_3",
        18: "UNBALANCED_CURRENTS",
        19: "BRK",
        20: "RESOLVER_LOT",
        21: "RESOLVER_DOS",
        22: "RESOLVER_LOS",
        23: "FLASH_CORRUPTION_APP_CFG",
        24: "FLASH_CORRUPTION_MC_CFG",
        25: "ENCODER_NO_MAGNET",
        26: "ENCODER_MAGNET_TOO_STRONG",
        27: "PHASE_FILTER",
        28: "ENCODER_FAULT",
        29: "LV_OUTPUT_FAULT",
        30: "ENCODER_SLIP",
        31: "OVERSPEED",
        32: "UNDERSPEED",
        33: "ABS_OVERSPEED",
    }


    # ==========================================================
    # INITIALISATION
    # ==========================================================

    def __init__(
        self,
        use_defaults=True,
        config=None,
    ):

        # Start with known defaults
        cfg = dict(self.DEFAULT_CONFIG)

        # Only apply custom settings when defaults are disabled
        if not use_defaults:

            if config is not None:
                cfg.update(config)

        self.config = cfg
        self.use_defaults = use_defaults

        self.uart_id = cfg["uart_id"]
        self.baudrate = cfg["baudrate"]

        self.tx_pin = cfg["tx_pin"]
        self.rx_pin = cfg["rx_pin"]

        self.min_duty = cfg["min_duty"]
        self.max_duty = cfg["max_duty"]

        self.max_current = cfg["max_current"]
        self.max_brake_current = cfg["max_brake_current"]

        self.max_rpm = cfg["max_rpm"]

        self.request_timeout_ms = cfg["request_timeout_ms"]

        self.debug = cfg["debug"]


        # UART interface
        self.uart = UART(
            self.uart_id,
            baudrate=self.baudrate,
            tx=Pin(self.tx_pin),
            rx=Pin(self.rx_pin),
            timeout=0,
            timeout_char=2,
        )


        # Incoming byte buffer
        self.rx_buffer = bytearray()


        self._debug("VESC UART initialised")


    # ==========================================================
    # DIAGNOSTICS
    # ==========================================================

    def _debug(self, message):

        if self.debug:
            print("[VESC UART]", message)


    def print_config(self):

        print()
        print("VESC UART Configuration")
        print("-----------------------")

        print(
            "Mode:",
            "DEFAULT" if self.use_defaults else "CUSTOM"
        )

        print("UART:", self.uart_id)
        print("Baud:", self.baudrate)

        print("TX GPIO:", self.tx_pin)
        print("RX GPIO:", self.rx_pin)

        print(
            "Duty range: {:.1f}% to {:.1f}%".format(
                self.min_duty * 100,
                self.max_duty * 100,
            )
        )

        print(
            "Max current: {:.1f} A".format(
                self.max_current
            )
        )

        print(
            "Max brake current: {:.1f} A".format(
                self.max_brake_current
            )
        )

        print(
            "Max RPM:",
            self.max_rpm
        )

        print()


    # ==========================================================
    # CRC
    # ==========================================================

    @staticmethod
    def crc16(data):

        crc = 0

        for byte in data:

            crc ^= byte << 8

            for _ in range(8):

                if crc & 0x8000:

                    crc = (
                        (crc << 1) ^ 0x1021
                    ) & 0xFFFF

                else:

                    crc = (
                        crc << 1
                    ) & 0xFFFF

        return crc


    # ==========================================================
    # NUMBER ENCODING
    # ==========================================================

    @staticmethod
    def _pack_i32(value):

        value = int(value) & 0xFFFFFFFF

        return bytes([
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])


    @staticmethod
    def _read_i16(data, index, scale=1.0):

        value = (
            (data[index] << 8)
            | data[index + 1]
        )

        if value & 0x8000:
            value -= 0x10000

        return value / scale, index + 2


    @staticmethod
    def _read_i32(data, index, scale=1.0):

        value = (
            (data[index] << 24)
            | (data[index + 1] << 16)
            | (data[index + 2] << 8)
            | data[index + 3]
        )

        if value & 0x80000000:
            value -= 0x100000000

        return value / scale, index + 4


    # ==========================================================
    # PACKET CREATION
    # ==========================================================

    def _make_packet(self, payload):

        length = len(payload)

        if length <= 255:

            header = bytes([
                2,
                length
            ])

        elif length <= 65535:

            header = bytes([
                3,
                (length >> 8) & 0xFF,
                length & 0xFF,
            ])

        else:

            header = bytes([
                4,
                (length >> 16) & 0xFF,
                (length >> 8) & 0xFF,
                length & 0xFF,
            ])


        crc = self.crc16(payload)


        return (
            header
            + payload
            + bytes([
                (crc >> 8) & 0xFF,
                crc & 0xFF,
                3,
            ])
        )


    # ==========================================================
    # TRANSMIT
    # ==========================================================

    def _send_command(self, command, data=b""):

        payload = bytes([command]) + data

        packet = self._make_packet(payload)

        written = self.uart.write(packet)

        if self.debug:

            print(
                "[VESC UART] TX",
                command,
                "|",
                written,
                "bytes"
            )

        return written


    # ==========================================================
    # RECEIVE PACKET DECODER
    # ==========================================================

    def _extract_packet(self):

        while len(self.rx_buffer) > 0:

            start = self.rx_buffer[0]


            # ------------------------------------------
            # Find valid packet start
            # ------------------------------------------

            if start not in (2, 3, 4):

                del self.rx_buffer[0]
                continue


            # ------------------------------------------
            # Determine packet length
            # ------------------------------------------

            if start == 2:

                if len(self.rx_buffer) < 2:
                    return None

                header_length = 2

                payload_length = (
                    self.rx_buffer[1]
                )


            elif start == 3:

                if len(self.rx_buffer) < 3:
                    return None

                header_length = 3

                payload_length = (
                    (self.rx_buffer[1] << 8)
                    | self.rx_buffer[2]
                )


            else:

                if len(self.rx_buffer) < 4:
                    return None

                header_length = 4

                payload_length = (
                    (self.rx_buffer[1] << 16)
                    | (self.rx_buffer[2] << 8)
                    | self.rx_buffer[3]
                )


            total_length = (
                header_length
                + payload_length
                + 3
            )


            # Wait for complete packet
            if len(self.rx_buffer) < total_length:
                return None


            payload_start = header_length

            payload_end = (
                payload_start
                + payload_length
            )


            # ------------------------------------------
            # Validate stop byte
            # ------------------------------------------

            if self.rx_buffer[payload_end + 2] != 3:

                del self.rx_buffer[0]
                continue


            payload = bytes(
                self.rx_buffer[
                    payload_start:payload_end
                ]
            )


            # ------------------------------------------
            # Validate CRC
            # ------------------------------------------

            received_crc = (
                self.rx_buffer[payload_end] << 8
            ) | (
                self.rx_buffer[payload_end + 1]
            )


            calculated_crc = self.crc16(payload)


            if calculated_crc != received_crc:

                self._debug("CRC error")

                del self.rx_buffer[0]
                continue


            # Remove packet from receive buffer
            del self.rx_buffer[:total_length]


            return payload


        return None


    # ==========================================================
    # RECEIVE
    # ==========================================================

    def _read_packet(self, timeout_ms=None):

        if timeout_ms is None:
            timeout_ms = self.request_timeout_ms


        start_time = time.ticks_ms()


        while (
            time.ticks_diff(
                time.ticks_ms(),
                start_time
            )
            < timeout_ms
        ):

            # Check existing buffer first
            packet = self._extract_packet()

            if packet is not None:
                return packet


            # Read UART
            if self.uart.any():

                data = self.uart.read()

                if data:
                    self.rx_buffer.extend(data)

            else:

                time.sleep_ms(1)


        return None


    def flush_rx(self):

        self.rx_buffer = bytearray()

        while self.uart.any():

            self.uart.read()


    # ==========================================================
    # REQUEST / RESPONSE
    # ==========================================================

    def _request(
        self,
        command,
        data=b"",
        timeout_ms=None,
    ):

        if timeout_ms is None:
            timeout_ms = self.request_timeout_ms


        self.flush_rx()

        self._send_command(
            command,
            data
        )


        start_time = time.ticks_ms()


        while (
            time.ticks_diff(
                time.ticks_ms(),
                start_time
            )
            < timeout_ms
        ):

            elapsed = time.ticks_diff(
                time.ticks_ms(),
                start_time
            )

            remaining = timeout_ms - elapsed

            packet = self._read_packet(
                remaining
            )


            if packet is None:
                return None


            if len(packet) > 0:

                if packet[0] == command:
                    return packet


        return None


    # ==========================================================
    # MOTOR CONTROL
    # ==========================================================

    def set_duty(self, duty):

        duty = max(
            self.min_duty,
            min(
                self.max_duty,
                float(duty)
            )
        )


        value = int(
            duty * 100000
        )


        self._send_command(
            self.COMM_SET_DUTY,
            self._pack_i32(value)
        )


        return duty


    def set_current(self, current):

        current = max(
            -self.max_current,
            min(
                self.max_current,
                float(current)
            )
        )


        value = int(
            current * 1000
        )


        self._send_command(
            self.COMM_SET_CURRENT,
            self._pack_i32(value)
        )


        return current


    def set_brake_current(self, current):

        current = max(
            0.0,
            min(
                self.max_brake_current,
                float(current)
            )
        )


        value = int(
            current * 1000
        )


        self._send_command(
            self.COMM_SET_CURRENT_BRAKE,
            self._pack_i32(value)
        )


        return current


    def set_rpm(self, rpm):

        rpm = int(
            max(
                -self.max_rpm,
                min(
                    self.max_rpm,
                    rpm
                )
            )
        )


        self._send_command(
            self.COMM_SET_RPM,
            self._pack_i32(rpm)
        )


        return rpm


    def set_position(self, degrees):

        value = int(
            float(degrees) * 1000000
        )


        self._send_command(
            self.COMM_SET_POS,
            self._pack_i32(value)
        )


        return degrees


    def stop(self):

        self.set_duty(0.0)

        self._debug(
            "Motor command set to zero"
        )


    # ==========================================================
    # GET TELEMETRY
    # ==========================================================

    def get_values(self, timeout_ms=None):

        packet = self._request(
            self.COMM_GET_VALUES,
            timeout_ms=timeout_ms
        )


        if packet is None:

            self._debug(
                "COMM_GET_VALUES timeout"
            )

            return None


        if len(packet) < 2:
            return None


        try:

            return self._parse_values(
                packet
            )

        except Exception as error:

            self._debug(
                "Telemetry parse error: {}".format(
                    error
                )
            )

            return None


    # ==========================================================
    # TELEMETRY PARSER
    # ==========================================================

    def _parse_values(self, packet):

        values = {}

        index = 1


        # ------------------------------------------
        # Temperature
        # ------------------------------------------

        values["temp_fet_c"], index = (
            self._read_i16(
                packet,
                index,
                10.0
            )
        )


        values["temp_motor_c"], index = (
            self._read_i16(
                packet,
                index,
                10.0
            )
        )


        # ------------------------------------------
        # Currents
        # ------------------------------------------

        values["motor_current_a"], index = (
            self._read_i32(
                packet,
                index,
                100.0
            )
        )


        values["input_current_a"], index = (
            self._read_i32(
                packet,
                index,
                100.0
            )
        )


        values["id_current_a"], index = (
            self._read_i32(
                packet,
                index,
                100.0
            )
        )


        values["iq_current_a"], index = (
            self._read_i32(
                packet,
                index,
                100.0
            )
        )


        # ------------------------------------------
        # Motor state
        # ------------------------------------------

        values["duty"], index = (
            self._read_i16(
                packet,
                index,
                1000.0
            )
        )


        values["rpm"], index = (
            self._read_i32(
                packet,
                index,
                1.0
            )
        )


        values["rpm"] = int(
            values["rpm"]
        )


        # ------------------------------------------
        # Input voltage
        # ------------------------------------------

        values["input_voltage_v"], index = (
            self._read_i16(
                packet,
                index,
                10.0
            )
        )


        # ------------------------------------------
        # Energy counters
        # ------------------------------------------

        values["amp_hours"], index = (
            self._read_i32(
                packet,
                index,
                10000.0
            )
        )


        values["amp_hours_charged"], index = (
            self._read_i32(
                packet,
                index,
                10000.0
            )
        )


        values["watt_hours"], index = (
            self._read_i32(
                packet,
                index,
                10000.0
            )
        )


        values["watt_hours_charged"], index = (
            self._read_i32(
                packet,
                index,
                10000.0
            )
        )


        # ------------------------------------------
        # Tachometer
        # ------------------------------------------

        values["tachometer"], index = (
            self._read_i32(
                packet,
                index,
                1.0
            )
        )


        values["tachometer_abs"], index = (
            self._read_i32(
                packet,
                index,
                1.0
            )
        )


        values["tachometer"] = int(
            values["tachometer"]
        )

        values["tachometer_abs"] = int(
            values["tachometer_abs"]
        )


        # ------------------------------------------
        # Fault code
        # ------------------------------------------

        values["fault_code"] = (
            packet[index]
        )

        index += 1


        values["fault"] = (
            self.FAULT_CODES.get(
                values["fault_code"],
                "UNKNOWN"
            )
        )


        # ------------------------------------------
        # PID position
        # ------------------------------------------

        values["position_deg"], index = (
            self._read_i32(
                packet,
                index,
                1000000.0
            )
        )


        # ------------------------------------------
        # Controller ID
        # ------------------------------------------

        if index < len(packet):

            values["controller_id"] = (
                packet[index]
            )

            index += 1


        # ------------------------------------------
        # Newer firmware:
        # Individual MOSFET temperatures
        # ------------------------------------------

        if index + 6 <= len(packet):

            values["temp_mos1_c"], index = (
                self._read_i16(
                    packet,
                    index,
                    10.0
                )
            )

            values["temp_mos2_c"], index = (
                self._read_i16(
                    packet,
                    index,
                    10.0
                )
            )

            values["temp_mos3_c"], index = (
                self._read_i16(
                    packet,
                    index,
                    10.0
                )
            )


        # ------------------------------------------
        # D-axis voltage
        # ------------------------------------------

        if index + 4 <= len(packet):

            values["vd"], index = (
                self._read_i32(
                    packet,
                    index,
                    1000.0
                )
            )


        # ------------------------------------------
        # Q-axis voltage
        # ------------------------------------------

        if index + 4 <= len(packet):

            values["vq"], index = (
                self._read_i32(
                    packet,
                    index,
                    1000.0
                )
            )


        # ------------------------------------------
        # Status
        # ------------------------------------------

        if index < len(packet):

            status = packet[index]

            values["status"] = status

            values["timeout_active"] = bool(
                status & 0x01
            )

            values["kill_switch_active"] = bool(
                status & 0x02
            )


        return values


    # ==========================================================
    # HUMAN-READABLE TELEMETRY
    # ==========================================================

    def print_values(self, values=None):

        if values is None:

            values = self.get_values()


        if values is None:

            print(
                "[VESC] No telemetry response"
            )

            return


        print(
            "[VESC] "
            "Vin={:.1f}V | "
            "RPM={} | "
            "Duty={:.1f}% | "
            "Motor={:.1f}A | "
            "Input={:.1f}A | "
            "FET={:.1f}C | "
            "MotorTemp={:.1f}C | "
            "Fault={}".format(

                values["input_voltage_v"],
                values["rpm"],
                values["duty"] * 100,

                values["motor_current_a"],
                values["input_current_a"],

                values["temp_fet_c"],
                values["temp_motor_c"],

                values["fault"],
            )
        )