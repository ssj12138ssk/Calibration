import pyads


plc = pyads.Connection('10.1.233.139.1.1', 855)
plc.open()
#plc.write_by_name("MAIN.i", 10, pyads.PLCTYPE_INT)
#running_state = plc.read_by_name("HumanInterface.TestHmiDataOut.planState", pyads.PLCTYPE_INT)
received_cmd = plc.read_by_name("HumanInterface.TestHmiDataOut.receivedMessage", pyads.PLCTYPE_REAL * 22)

print(received_cmd)

plc.close()