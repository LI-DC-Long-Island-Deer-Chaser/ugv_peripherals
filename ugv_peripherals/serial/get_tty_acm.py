import os
import subprocess

def get_linux_command_output(command_to_run):
    return subprocess.check_output(command_to_run, shell=True, text=True).strip()


def get_serial():
    candidates = [i for i in os.listdir("/dev/") if i.startswith("ttyACM")]
    candidates_dict = dict()
    for c in candidates:
        result = get_linux_command_output("readlink -f /sys/class/tty/%s"%(c))+"/../../../"
        candidates_dict[c] = result


        target = get_linux_command_output("lsusb | grep -i QinHeng").split("QinHeng")[0].strip().split()[-1].replace(' ',':')

    # print(f"SEARCHING FOR {target}")

    # print(candidates_dict)

    for keys, values in candidates_dict.items():
        candidates_dict[keys] = get_linux_command_output(f"cat {values}/idVendor")+':'+get_linux_command_output("cat %s"%(values)+"idProduct")

        # print(keys, values, candidates_dict[keys])

        if candidates_dict[keys] == target:
            return keys
