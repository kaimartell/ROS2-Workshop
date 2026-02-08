import json
import logging
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional
from urllib.parse import urlparse

from host_agent.backends import create_backend


logging.basicConfig(level=logging.INFO, format="[host_agent] %(asctime)s %(levelname)s: %(message)s")


class HostAgentHTTPServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        backend_name: str,
        backend_kwargs: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(server_address, HostAgentRequestHandler)
        self.backend = create_backend(backend_name, **(backend_kwargs or {}))

    def backend_health(self) -> Dict[str, Any]:
        if hasattr(self.backend, "health"):
            health = self.backend.health()
            if isinstance(health, dict):
                return health
        return {
            "ok": True,
            "backend": getattr(self.backend, "name", "unknown"),
            "spike_connected": True,
        }

    def close_backend(self) -> None:
        if hasattr(self.backend, "close"):
            try:
                self.backend.close()
            except Exception:  # noqa: BLE001
                logging.exception("Backend close() failed")


class HostAgentRequestHandler(BaseHTTPRequestHandler):
    server: HostAgentHTTPServer

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            if path == "/health":
                self._write_json(HTTPStatus.OK, self.server.backend_health())
                return

            if path == "/state":
                self._write_json(HTTPStatus.OK, self.server.backend.get_state())
                return

            self._write_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
        except Exception as exc:  # noqa: BLE001
            logging.exception("GET %s failed", path)
            self._write_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            payload = self._read_json_body()
        except ValueError as exc:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return

        try:
            if path == "/motor/run":
                speed = float(payload.get("speed", 0.0))
                duration = float(payload.get("duration", 0.0))
                try:
                    result = self.server.backend.run(speed=speed, duration=duration)
                except Exception:  # noqa: BLE001
                    logging.exception("Backend run() failed")
                    self._write_json(
                        HTTPStatus.OK, {"accepted": False, "error": "backend_run_exception"}
                    )
                    return

                if isinstance(result, dict):
                    accepted = bool(result.get("accepted", False))
                    response: Dict[str, Any] = {"accepted": accepted}
                    if "error" in result:
                        response["error"] = str(result["error"])
                    if "note" in result:
                        response["note"] = str(result["note"])
                    self._write_json(HTTPStatus.OK, response)
                else:
                    self._write_json(HTTPStatus.OK, {"accepted": True})
                return

            if path == "/motor/stop":
                try:
                    result = self.server.backend.stop()
                except Exception:  # noqa: BLE001
                    logging.exception("Backend stop() failed")
                    self._write_json(
                        HTTPStatus.OK, {"stopped": False, "error": "backend_stop_exception"}
                    )
                    return

                if isinstance(result, dict):
                    stopped = bool(result.get("stopped", False))
                    response = {"stopped": stopped}
                    if "error" in result:
                        response["error"] = str(result["error"])
                    self._write_json(HTTPStatus.OK, response)
                else:
                    self._write_json(HTTPStatus.OK, {"stopped": True})
                return

            self._write_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
        except Exception as exc:  # noqa: BLE001
            logging.exception("POST %s failed", path)
            self._write_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})

    def _read_json_body(self) -> Dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0:
            return {}

        body = self.rfile.read(content_length)
        if not body:
            return {}

        parsed = json.loads(body.decode("utf-8"))
        if isinstance(parsed, dict):
            return parsed
        raise ValueError("JSON body must be an object")

    def _write_json(self, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        logging.info("%s - %s", self.address_string(), format % args)

    def log_error(self, format: str, *args: Any) -> None:  # noqa: A003
        # Keep noisy TLS/garbage probe traffic out of workshop INFO logs.
        logging.debug("%s - %s", self.address_string(), format % args)


def run_server(
    host: str,
    port: int,
    backend_name: str,
    backend_kwargs: Optional[Dict[str, Any]] = None,
) -> None:
    server = HostAgentHTTPServer((host, port), backend_name=backend_name, backend_kwargs=backend_kwargs)
    logging.info("Starting host agent on http://%s:%s with backend=%s", host, port, backend_name)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Shutting down host agent")
    finally:
        server.close_backend()
        server.shutdown()
        server.server_close()
