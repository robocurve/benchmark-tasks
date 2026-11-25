"""Launch the pinned CloudXR runtime with one IPv4 WSS listener.

The Quest URL uses an IPv4 address. Avoid dual-stack bind ambiguity without
enabling port sharing or interfering with an existing listener.
"""
import isaacteleop.cloudxr.wss as wss
import isaacteleop.cloudxr.runtime as runtime
import isaacteleop.cloudxr.launcher as launcher
from functools import partial

# Shared GPUs may take longer than the SDK's 30 seconds to create Vulkan/NVENC.
runtime.RUNTIME_STARTUP_TIMEOUT_SEC=120
launcher.RUNTIME_STARTUP_TIMEOUT_SEC=120
launcher.wait_for_runtime_ready_sync=partial(runtime.wait_for_runtime_ready_sync,timeout_sec=120)

original_serve=wss.ws_serve
def ipv4_serve(*args,**kwargs):
    kwargs['host']='0.0.0.0'
    return original_serve(*args,**kwargs)
wss.ws_serve=ipv4_serve

if __name__=='__main__':
    from isaacteleop.cloudxr.__main__ import main
    main()
