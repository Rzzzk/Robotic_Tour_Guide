#!/usr/bin/env python3
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import json
import os
import wave
import numpy as np

class AudioReceiver:
    def __init__(self):
        rospy.init_node('audio_receiver', anonymous=True)
        
        self.save_directory = './received_audio'
        self.current_file = None
        self.wav_writer = None
        self.metadata = None
        self.received_chunks = []
        
        # create save directory if needed
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        self.save_complete_pub = rospy.Publisher('/audio_save_complete', String, queue_size=10)
        self.robot_state_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        
        # subscribe to audio stream and metadata
        rospy.Subscriber('/audio_metadata', String, self.metadata_callback)
        rospy.Subscriber('/audio_stream', AudioData, self.audio_callback)
        
        rospy.loginfo("Audio receiver initialized")

    def metadata_callback(self, msg):
        """handle incoming audio metadata"""
        try:
            self.metadata = json.loads(msg.data)
            rospy.loginfo(f"Receiving audio: {self.metadata['fileName']}")
            
            # close any existing file before receiving new
            if self.wav_writer:
                self.wav_writer.close()
            
            # reset for new file
            self.received_chunks = []
            
        except Exception as e:
            rospy.logerr(f"Error processing metadata: {e}")

    def audio_callback(self, msg):
        """handle incoming audio chunks"""
        if not msg.data:  # empty message signals means end of file
            self.save_audio_file()
            return
            
        if not self.metadata:
            return
            
        # store chunk
        self.received_chunks.append(bytes(msg.data))

    def save_audio_file(self):
        """save complete audio file"""
        if not self.metadata or not self.received_chunks:
            return
            
        try:
            file_path = os.path.join(self.save_directory, self.metadata['fileName'])
            
            # write chunks directly to file
            with open(file_path, 'wb') as f:
                for chunk in self.received_chunks:
                    f.write(chunk)

            # publish save completion message
            save_msg = "done saving"
            self.save_complete_pub.publish(save_msg)
    
            rospy.loginfo(f"Audio file saved: {file_path}")
            
            # reset for next file
            self.metadata = None
            self.received_chunks = []
            
        except Exception as e:
            rospy.logerr(f"Error saving audio file: {e}")

if __name__ == '__main__':
    try:
        receiver = AudioReceiver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass