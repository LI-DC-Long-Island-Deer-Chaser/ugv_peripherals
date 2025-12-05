import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger 
from rclpy.callback_groups import ReentrantCallbackGroup 
from rclpy.executors import MultiThreadedExecutor 
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
from random import choice
# from . import custom_play_audio
import getpass
from subprocess import Popen, PIPE
from re import findall

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

# I imported it here because I cannot for the life of me get it working.
def play_audio(audio_file):
    card_num, device_num = get_card_device_num()
    print(audio_file)
    audio_play = Popen(['aplay', '-D', f'plughw:{card_num},{device_num}', f'{audio_file}'])

class SpeakerNode(Node):
    def __init__(self):
        super().__init__("speaker")

        self.callback_group_ = ReentrantCallbackGroup()

        # get root location of resources
        package_share_dir_ = Path(get_package_share_directory('ugv_peripherals'))
        # get location of audio files 
        wav_audio_folder_path_ = package_share_dir_ / "resource" / "audio_lists_wav"
        # get the paths of all the audio files 
        self.wav_audio_files_lst_ = []
        for audio_file_wav in os.listdir(wav_audio_folder_path_):
            full_path_wav = os.path.join(wav_audio_folder_path_, audio_file_wav)
            self.wav_audio_files_lst_.append(full_path_wav)
        print(self.wav_audio_files_lst_)
        # service server
        self.audio_service_ = self.create_service(
            Trigger, 
            '/audio/play_sound', 
            self.handle_play_sound_, 
            callback_group=self.callback_group_
        )

        # situational awareness logs
        self.get_logger().info('Speaker Service started')
        self.get_logger().info(f"Running as: {getpass.getuser()}")

    def handle_play_sound_(self, request, response): 
        chosen_wav = choice(self.wav_audio_files_lst_)
        play_audio(chosen_wav)
        response.success = True
        response.message = "Audio played successfully"
        return response


def main(args=None):
    rclpy.init(args=args)

    speaker_node = SpeakerNode()

    # use MultiThreadedExecutor to handle concurrent service calls
    executor = MultiThreadedExecutor()
    executor.add_node(speaker_node)

    try:    
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally: 
        speaker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
