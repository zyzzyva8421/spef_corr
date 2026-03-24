"""
pytest configuration for spef_rc_correlation tests.

Adds a --spef-path command-line option so that an optional real SPEF file
can be supplied when running the integration tests:

    pytest --spef-path /path/to/20-blabla.spef

Integration tests that require a real SPEF file are automatically skipped
when --spef-path is not provided.
"""

import pytest


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--spef-path",
        action="store",
        default=None,
        metavar="PATH",
        help="Path to a real SPEF file used by integration tests (e.g. 20-blabla.spef).",
    )


@pytest.fixture
def real_spef_path(request: pytest.FixtureRequest) -> str:
    """Return the path passed via --spef-path, or skip the test if not provided."""
    path = request.config.getoption("--spef-path")
    if path is None:
        pytest.skip("No --spef-path provided; skipping real-SPEF integration test.")
    return path
