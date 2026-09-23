# Companion dependency patch

The validated Pi checkout has the attached aioice fix applied to its `go2` submodule.
It makes a queued STUN retry return if its future is already complete, avoiding an
exception when cancellation or a response wins the timer race during shutdown.

After initializing submodules, apply it once from the SDK repository root:

```bash
git -C go2_robot_sdk/external_lib/aioice apply --check ../../../patches/aioice-cancelled-stun-retry.patch
git -C go2_robot_sdk/external_lib/aioice apply ../../../patches/aioice-cancelled-stun-retry.patch
```

If the reverse check succeeds, the patch is already applied. Reinstall aioice if
your Python environment uses a non-editable installation. The upstream submodule
reference is unchanged; the tested patch is preserved here rather than pointing to
an unpublished upstream commit.
