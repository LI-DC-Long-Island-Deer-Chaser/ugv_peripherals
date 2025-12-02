import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger 
from rclpy.callback_groups import ReentrantCallbackGroup 
from rclpy.executors import MultiThreadedExecutor 
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
from random import choice
from speaker_controller.custom_play_audio import play_audio
import getpass

class SpeakerNode(Node):
    def __init__(self):
        super().__init__("speaker_node")

        self.callback_group_ = ReentrantCallbackGroup()

        # get root location of resources
        package_share_dir_ = Path(get_package_share_directory('speaker_controller'))
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
