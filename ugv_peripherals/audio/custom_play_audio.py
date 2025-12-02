from subprocess import Popen, PIPE
from re import findall
import os

def get_card_device_num():
    # aplay -l | grep USB
    # get the card and device number and store in variable
    aplay = Popen(['aplay', '-l'], stdout=PIPE, stderr=PIPE)
    grep = Popen(['grep', 'USB'], stdin=aplay.stdout, stdout=PIPE, stderr=PIPE, text=True)
    aplay.stdout.close()
    stdout, stderr = grep.communicate()

    card_num = '-1'
    device_num = '-1'

    if stdout.strip():
        print("USB devices found:")
        print(stdout)
        card_num, device_num = findall(r'\d+', stdout)
    else:
        print("No USB devices found.")

    return [card_num, device_num]

# aplay -D plughw:[card_num],[device_num] [file]
def play_audio(audio_file):
    card_num, device_num = get_card_device_num()
    print(audio_file)
    audio_play = Popen(['aplay', '-D', f'plughw:{card_num},{device_num}', f'{audio_file}'])
