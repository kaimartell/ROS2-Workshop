"""Repo-root execution shim for `python -m host_agent`.

This repository uses `host_agent/host_agent` as the actual Python package.
When running from the repo root, `python -m host_agent` resolves this outer
directory first. We bridge that case here so docs and dry-run commands work.
"""

from pathlib import Path


def _run_inner_main() -> None:
    import host_agent as outer_pkg

    inner_pkg_dir = Path(__file__).resolve().parent / "host_agent"
    if inner_pkg_dir.is_dir():
        outer_pkg.__path__.append(str(inner_pkg_dir))

    from host_agent.host_agent.__main__ import main

    main()


if __name__ == "__main__":
    _run_inner_main()
