import openamp
import refclk
import machine
import struct

def task_callback(src_addr, data):
    now_us = refclk.now_us()
    print("time diff", now_us - int.from_bytes(data, "little"))


@openamp.async_remote(task_callback)
async def task1(ept):
    import asyncio
    import refclk
    while True:
        now_us = refclk.now_us()
        ept.send(now_us.to_bytes(4, "little"))
        await asyncio.sleep(1)


refclk.enable()
rproc = openamp.RemoteProc(0x80320000)
rproc.start()

while True:
    machine.idle()
