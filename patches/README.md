# Companion dependency patch

`aioice-cancelled-stun-retry.patch` makes a queued STUN retry return if its future
is already complete, avoiding an exception when cancellation or a response wins the
timer race during shutdown.

aioice is vendored at `go2_robot_sdk/external_lib/aioice` with this patch already
applied (see `VENDORED.md` there), so there is no submodule to initialise and no
patch to apply. The file is kept as a record of the local change against upstream
`ff5755a`, and so it can be re-applied when the vendored copy is updated.
