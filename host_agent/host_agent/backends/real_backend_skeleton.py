import logging

from host_agent.backends.spike_ble_backend import SpikeBleBackend


class RealSpikeBackend(SpikeBleBackend):
    """
    Backward-compatible wrapper around the real BLE backend.

    TODO notes for future hardening:
    - Replace low-level LWP packets with a dedicated SPIKE motor library if desired.
    - Add per-hub motor-port discovery rather than static A-F mapping.
    - Expose richer diagnostics (battery, RSSI, firmware).
    """

    name = "spike_ble"

    def __init__(self, **kwargs):
        logging.warning("'real' backend alias is deprecated. Use --backend spike_ble.")
        super().__init__(**kwargs)
