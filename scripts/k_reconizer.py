#!/usr/bin/python
import os
import subprocess
from pickle import DICT
from pocketsphinx import LiveSpeech, get_model_path, AudioFile, get_data_path, DefaultConfig, Decoder
from time import sleep
import pyaudio
import rospy
from std_msgs.msg import String


class SpeechMessage(object):
    """Class to publish audio to topic"""

    def __init__(self):
        # Params
        self._input = "~input"
        self._hmm = "~hmm"
        self._dict = "~dict"
        self._lm = "~lm"
        # _rate_bool = False

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("speech_recog_data", String, queue_size=10)

        # initialize node
        rospy.init_node("Live_recognizer")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)
        # self.i = 1
        self.model_path = get_model_path()
        self.package_path = subprocess.run(
            ['rospack', 'find', 'museum_nav_bot'], stdout=subprocess.PIPE).stdout.decode('utf-8')
        self.data_path = os.path.join(self.package_path, 'speech_info')
        self.config = {}
        self.config['hmm'] = os.path.join(self.model_path, 'en-us')
        self.config['lm'] = os.path.join(self.model_path, 'en-us.lm.bin')
        self.config['dict'] = os.path.join(
            self.model_path, 'cmudict-en-us.dict')
        self.config['buffer_size'] = 2048
        self.config['verbose'] = False
        self.config['no_search'] = False
        self.config['full_utt'] = False
        # decoder = Decoder(config)
        self.speech = self.get_reconizer()

        rospy.loginfo("Now listening....")
        # sleep(5)

        # All set. Publish to topic
        self.transfer_speech_msg()

    def get_reconizer(self):
        # Checking if speech profile file given or default is needed
        if rospy.has_param(self._hmm):
            if rospy.get_param(self._hmm) != ":default":
                self.config['hmm'] = os.path.abspath(os.path.join(
                    self.data_path, rospy.get_param(self._hmm)))

        if rospy.has_param(self._dict):
            if rospy.get_param(self._dict) != ":default":
                self.config['dict'] = os.path.abspath(os.path.join(
                    self.data_path, rospy.get_param(self._dict)))

        if rospy.has_param(self._lm):
            if rospy.get_param(self._lm) != ":default":
                self.config['lm'] = os.path.abspath(os.path.join(
                    self.data_path, rospy.get_param(self._lm)))

        print("hmm:", self.config['hmm'])
        print("dict:", self.config['dict'])
        print("lm:", self.config['lm'])
        # Checking if audio file given or system microphone is needed
        if rospy.has_param(self._input):
            if rospy.get_param(self._input) != ":default":
                # _rate_bool = True
                # self.stream = open(rospy.get_param(self._input), 'rb')
                # rate = rospy.Rate(5)  # 10hz
                self.config['audio_file'] = os.path.abspath(os.path.join(
                    self.package_path, rospy.get_param(self._input)))
                return AudioFile(**self.config)
            else:
                # Initializing pyaudio for input from system microhpone
                # self.stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                #                                      rate=16000, input=True, frames_per_buffer=1024)
                # self.stream.start_stream()
                self.config['sampling_rate'] = 16000
                return LiveSpeech(**self.config)
        else:
            rospy.logerr(
                "No input means provided. Please use the launch file instead")

    def transfer_speech_msg(self):
        """Function to publish input audio to topic"""

        while not rospy.is_shutdown():
            for phrase in self.speech:
                b1 = str(phrase)
                print("Echo: ", b1)
                # Publish speech to topic
                self.pub_.publish(b1)
                # rospy.loginfo("COUNT" + str(self.i))
                # self.i += 1
            # buf = self.stream.read(1024)
            # if buf:
            #     self.decoder.start_utt()
            #     self.decoder.process_raw(buf, False, False)
            #     self.decoder.end_utt()
            #     b1 = str([seg.word for seg in self.decoder.seg()], "utf-8")
            #     print('Best hypothesis segments:', b1)
            #     # Publish speech to topic
            #     self.pub_.publish(b1)
            #     rospy.loginfo("COUNT" + str(self.i))
            #     self.i += 1

            # else:
            #     rospy.loginfo("Buffer returned null")
            #     break

    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop Regconizer")
        rospy.sleep(1)


if __name__ == "__main__":
    SpeechMessage()
