import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from std_msgs.msg import String
from rclpy.time import Duration


class MusicPlayer(Node):
    def __init__(self):
        super().__init__('music_player')

        # Publisher
        self.music_publisher = self.create_publisher(AudioNoteVector, '/cmd_audio', 1)

        # Subscriber for LLM events
        self.event_sub = self.create_subscription(
            String,
            '/llm_audio_event',
            self.handle_audio_event,
            10
        )

        # Timer for background music
        self.music_player = self.create_timer(1, self.play_music)

        self.delay_counter = 0
        self.song_length = 28

        # Prevent music from overriding chimes
        self.pause_music_ticks = 0

        # Prevent duplicate chimes
        self.last_event = None


        freq_a_very_low = 110
        freq_b_very_low = 123
        freq_c_very_low = 131
        freq_d_very_low = 147
        freq_e_very_low = 165
        freq_f_very_low = 175
        freq_g_very_low = 196

        freq_a_low = 220
        freq_b_low = 247
        freq_c_low = 262
        freq_d_low = 294

        freq_a = 440
        freq_b = 494
        freq_c = 523
        freq_d = 587
        freq_e = 659
        freq_f = 698
        freq_g = 784

        freq_a_high = 880
        freq_b_high = 988
        freq_c_high = 1047
        freq_d_high = 1175
        freq_e_high = 1319

        rest = 0

        dur_half = Duration(seconds=0.6).to_msg()
        dur_quarter = Duration(seconds=0.3).to_msg()
        dur_short = Duration(seconds=0.15).to_msg()

        # Helper for short notes
        def short(freq):
            return AudioNote(frequency=freq, max_runtime=dur_short)


        self.chime_thinking = AudioNoteVector(
            append=False,
            notes=[
                short(freq_c),
                short(freq_e),
            ]
        )

        self.chime_response = AudioNoteVector(
            append=False,
            notes=[
                short(freq_e),
                short(freq_g),
                short(freq_c_high),
            ]
        )

        self.chime_fallback = AudioNoteVector(
            append=False,
            notes=[
                short(freq_g_very_low),
                short(freq_c_very_low),
            ]
        )

   
        rest_quarter = AudioNote(frequency=rest, max_runtime=dur_quarter)

        note_c_low_quarter = AudioNote(frequency=freq_c_low, max_runtime=dur_quarter)
        note_a_low_quarter = AudioNote(frequency=freq_a_low, max_runtime=dur_quarter)
        note_b_low_quarter = AudioNote(frequency=freq_b_low, max_runtime=dur_quarter)
        note_d_very_low_quarter = AudioNote(frequency=freq_d_very_low, max_runtime=dur_quarter)
        note_f_very_low_quarter = AudioNote(frequency=freq_f_very_low, max_runtime=dur_quarter)
        note_g_very_low_quarter = AudioNote(frequency=freq_g_very_low, max_runtime=dur_quarter)

        note_c_low_half = AudioNote(frequency=freq_c_low, max_runtime=dur_half)
        note_g_very_low_half = AudioNote(frequency=freq_g_very_low, max_runtime=dur_half)

        note_e_quarter = AudioNote(frequency=freq_e, max_runtime=dur_quarter)
        note_d_quarter = AudioNote(frequency=freq_d, max_runtime=dur_quarter)
        note_c_quarter = AudioNote(frequency=freq_c, max_runtime=dur_quarter)
        note_g_quarter = AudioNote(frequency=freq_g, max_runtime=dur_quarter)
        note_d_half = AudioNote(frequency=freq_d, max_runtime=dur_half)


        self.song = AudioNoteVector(
            append=False,
            notes=[
                note_c_low_quarter,
                note_a_low_quarter,
                note_c_low_quarter,
                note_b_low_quarter,
                note_a_low_quarter,
                note_g_very_low_half,

                rest_quarter,
                note_a_low_quarter,
                note_d_very_low_quarter,
                note_a_low_quarter,
                note_f_very_low_quarter,
                note_g_very_low_quarter,
                note_c_low_half,
                rest_quarter,

                note_e_quarter,
                rest_quarter,
                note_e_quarter,
                note_d_quarter,
                note_c_quarter,
                note_g_quarter,
                note_d_half,
                rest_quarter,
            ]
        )

    def play_music(self):
        if self.pause_music_ticks > 0:
            self.pause_music_ticks -= 1
            return

        if self.delay_counter <= 0:
            self.delay_counter = self.song_length
            self.music_publisher.publish(self.song)

        self.delay_counter -= 1

    def handle_audio_event(self, msg: String):
        event = msg.data

        # Prevent duplicate triggers
        if event == self.last_event:
            return

        self.last_event = event

        # Pause music briefly so chime is heard
        self.pause_music_ticks = 2

        if event == "Thinking":
            self.music_publisher.publish(self.chime_thinking)

        elif event == "Response":
            self.music_publisher.publish(self.chime_response)

        elif event == "Failure":
            self.music_publisher.publish(self.chime_fallback)


def main():
    rclpy.init()
    music_player = MusicPlayer()
    rclpy.spin(music_player)


if __name__ == '__main__':
    main()