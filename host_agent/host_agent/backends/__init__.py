import inspect
from typing import Any, Callable, Dict, List

from host_agent.backends.mock_backend import MockBackend
from host_agent.backends.spike_ble_backend import SpikeBleBackend, discover_spike_ble_devices
from host_agent.backends.spike_usb_backend import SpikeUsbBackend, list_serial_ports

BackendFactory = Callable[..., Any]

_BACKEND_FACTORIES: Dict[str, BackendFactory] = {
    "mock": MockBackend,
    "spike_ble": SpikeBleBackend,
    "spike_usb": SpikeUsbBackend,
    # Backward-compatible alias for older docs/scripts.
    "real": SpikeBleBackend,
}


def create_backend(name: str, **kwargs: Any) -> Any:
    normalized = name.strip().lower()
    factory = _BACKEND_FACTORIES.get(normalized)
    if factory is None:
        supported = ", ".join(sorted(_BACKEND_FACTORIES.keys()))
        raise ValueError(f"Unsupported backend: {name}. Supported backends: {supported}")

    signature = inspect.signature(factory)
    accepts_var_kwargs = any(
        param.kind == inspect.Parameter.VAR_KEYWORD
        for param in signature.parameters.values()
    )
    if accepts_var_kwargs:
        return factory(**kwargs)

    accepted_names = set(signature.parameters.keys())
    filtered_kwargs = {key: value for key, value in kwargs.items() if key in accepted_names}
    return factory(**filtered_kwargs)


def list_backends() -> List[str]:
    return ["mock", "spike_ble", "spike_usb"]


def list_ble_devices(timeout: float = 4.0, name_filter: str = "") -> List[Dict[str, Any]]:
    return discover_spike_ble_devices(timeout=timeout, name_filter=name_filter)


def list_serial_devices() -> List[Dict[str, Any]]:
    return list_serial_ports()
