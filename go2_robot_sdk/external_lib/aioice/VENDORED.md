# Vendored aioice

This directory used to be a git submodule. It is now plain files so that a clean
checkout of this repository is exactly the code the Pi runs.

- Upstream: https://github.com/legion1581/aioice (branch `go2`)
- Upstream commit: `ff5755a1e37127411b5fc797c105804db8437445`
- Local change: `patches/aioice-cancelled-stun-retry.patch` (already applied here)
- Kept: `src/aioice/`, `LICENSE`, `README.rst`. `setup.py` installs only `src/aioice/*`.

To update: copy `src/aioice/` from a newer upstream commit, re-apply the patch if it
is still needed, and update the commit above.
