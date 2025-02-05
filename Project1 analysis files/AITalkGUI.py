#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import json
from io import StringIO

import sys
import time
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class InitThread(QThread):

    def run(self):
        time.sleep(1)
        
class MainWindow(QWidget):

    signal_edittextupdate = pyqtSignal()
    signal_widgetupdate = pyqtSignal()

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        rospy.init_node('tts_client')
        # Publisher
        self.pub = rospy.Publisher('tts', String, queue_size=10)
        # Subscriber
        z = rospy.Subscriber('tts_return', String, self.sub_callback)
        
        # TTS ID
        self.TTSID = 0
        
        # 音声合成話者parameters
        self.ParamVoiceName = 'maki_emo_22_standard'
        self.ParamVolume = 1.0
        self.ParamRate = 1.0
        self.ParamPitch = 1.0
        self.ParamEmphasis = 1.0
        self.ParamEmotionJoy = 0.0
        self.ParamEmotionAngry = 0.0
        self.ParamEmotionSad = 0.0
        self.LocalAudioPlayerFlag = 1
        self.ExternalAudioPlayerFlag = 0

        initthread = InitThread()
        initthread.finished.connect(self.on_click_node)
        initthread.start()
        
        self.InitUI()
        
    def sub_callback(self, data):
        self.recv_dict = json.loads(data.data)
        # Qtではすべての描画がメインスレッドから行われる必要がある
        # そのため，ここでQPlainTextEditに文字を入力しようとすると落ちる
        #self.edtrcv.setPlainText(json.dumps(recv_dict, indent=2))
        self.signal_edittextupdate.emit()

        if self.recv_dict['Message'] == "SystemParam":
            if ('Speaker' in self.recv_dict) == True:
                self.ParamVoiceName = self.recv_dict['Speaker']
            if ('Volume' in self.recv_dict) == True:
                self.ParamVolume = self.recv_dict['Volume']
            if ('Rate' in self.recv_dict) == True:
                self.ParamRate = self.recv_dict['Rate']
            if ('Pitch' in self.recv_dict) == True:
                self.ParamPitch = self.recv_dict['Pitch']
            if ('Emphasis' in self.recv_dict) == True:
                self.ParamEmphasis = self.recv_dict['Emphasis']
            if ('Emotion' in self.recv_dict) == True:
                if ('Joy' in self.recv_dict['Emotion']) == True:
                    self.ParamEmotionJoy = self.recv_dict['Emotion']['Joy']
                if ('Angry' in self.recv_dict['Emotion']) == True:
                    self.ParamEmotionAngry = self.recv_dict['Emotion']['Angry']
                if ('Sad' in self.recv_dict['Emotion']) == True:
                    self.ParamEmotionSad = self.recv_dict['Emotion']['Sad']
            if ('LocalAudioPlayerFlag' in self.recv_dict) == True:
                self.LocalAudioPlayerFlag = self.recv_dict['LocalAudioPlayerFlag']
            if ('ExternalAudioPlayerFlag' in self.recv_dict) == True:
                self.ExternalAudioPlayerFlag = self.recv_dict['ExternalAudioPlayerFlag']
            self.signal_widgetupdate.emit()
                
    def InitUI(self):        
        # Window
        self.setGeometry(300, 150, 500, 500)
        self.setWindowTitle('AITalk Client')

        # AppにTextEditを更新する命令を設定する
        self.signal_edittextupdate.connect(self.on_edit_text)
        # AppにSlider等を更新する命令を設定する
        self.signal_widgetupdate.connect(self.on_widget_update)
        
        # text edit for text to speech
        self.edttts = QPlainTextEdit('ここに言葉を入力してください。', self)
        labntts = QLabel("Text to speech", self)
        # text edit for message from server
        self.edtrcv = QPlainTextEdit(self)
        labnrcv = QLabel("Message from server", self)

        edtlayout = QVBoxLayout()
        edtlayout.addWidget(labntts)
        edtlayout.addWidget(self.edttts)
        edtlayout.addWidget(labnrcv)
        edtlayout.addWidget(self.edtrcv)

        # button to start TTS
        btntts = QPushButton('TTS Start', self)
        btntts.clicked.connect(self.on_click_tts)
        # button to stop TTS
        btnstp = QPushButton('TTS Stop', self)
        btnstp.clicked.connect(self.on_click_stop)
        btnlayout = QHBoxLayout()
        btnlayout.addWidget(btntts)
        btnlayout.addWidget(btnstp)

        # checkbox to toggle carlos audio player
        self.chkcplayer = QCheckBox('Carlos Player', self)
        self.chkcplayer.stateChanged.connect(self.on_check_cplayer)
        if self.ExternalAudioPlayerFlag == 1:
            self.chklplayer.toggle()
        # checkbox to toggle local audio player
        self.chklplayer = QCheckBox('Local Player', self)
        if self.LocalAudioPlayerFlag == 1:
            self.chklplayer.toggle()
        self.chklplayer.stateChanged.connect(self.on_check_lplayer)
        # button to close
        btncls = QPushButton('Close', self)
        btncls.clicked.connect(self.on_click_close)
        chklayout = QHBoxLayout()
        chklayout.addWidget(self.chkcplayer)
        chklayout.addWidget(self.chklplayer)
        chklayout.addWidget(btncls)
        
        #speaker select
        self.combospk = QComboBox(self)
        self.combospk.addItem("maki_emo_22_standard")
        self.combospk.addItem("maki_emo_16_standard")
        self.combospk.activated[str].connect(self.on_speaker_changed)
        labnspk = QLabel("Speaker", self)
        combospklayout = QHBoxLayout()
        combospklayout.addWidget(labnspk)
        combospklayout.addWidget(self.combospk)
        
        # Volume
        self.sldvol = QSlider(Qt.Horizontal, self)
        self.sldvol.setRange(0, 200)  # 0 - 2.00
        self.sldvol.setValue(100)     # 1.00
        self.sldvol.valueChanged.connect(self.on_sldvol)
        self.labvvol = QLabel("%.2f" % self.ParamVolume, self)
        labnvol = QLabel("Volume", self)
        sldvollayout = QHBoxLayout()
        sldvollayout.addWidget(labnvol)
        sldvollayout.addWidget(self.sldvol)
        sldvollayout.addWidget(self.labvvol)
        
        # Rate
        self.sldrate = QSlider(Qt.Horizontal, self)
        self.sldrate.setRange(50, 400) # 0.5 - 4.00
        self.sldrate.setValue(100)     # 1.00
        self.sldrate.valueChanged.connect(self.on_sldrate)
        self.labvrate = QLabel("%.2f" % self.ParamRate, self)
        labnrate = QLabel("Rate", self)
        sldratelayout = QHBoxLayout()
        sldratelayout.addWidget(labnrate)
        sldratelayout.addWidget(self.sldrate)
        sldratelayout.addWidget(self.labvrate)

        # Pitch
        self.sldpt = QSlider(Qt.Horizontal, self)
        self.sldpt.setRange(50, 200)   # 0.5 - 2.00
        self.sldpt.setValue(100)       # 1.00
        self.sldpt.valueChanged.connect(self.on_sldpt)
        self.labvpt = QLabel("%.2f" % self.ParamPitch, self)
        labnpt = QLabel("Pitch", self)
        sldptlayout = QHBoxLayout()
        sldptlayout.addWidget(labnpt)
        sldptlayout.addWidget(self.sldpt)
        sldptlayout.addWidget(self.labvpt)

        # Emphasis
        self.sldem = QSlider(Qt.Horizontal, self)
        self.sldem.setRange(0, 200)    # 0.0 - 2.00
        self.sldem.setValue(100)       # 1.00
        self.sldem.valueChanged.connect(self.on_sldem)
        self.labvem = QLabel("%.2f" % self.ParamEmphasis, self)
        labnem = QLabel("Emphasis", self)
        sldemlayout = QHBoxLayout()
        sldemlayout.addWidget(labnem)
        sldemlayout.addWidget(self.sldem)
        sldemlayout.addWidget(self.labvem)

        # EmotionJoy
        self.sldjoy = QSlider(Qt.Horizontal, self)
        self.sldjoy.setRange(0, 100)    # 0.0 - 1.00
        self.sldjoy.setValue(0)         # 0.0
        self.sldjoy.valueChanged.connect(self.on_sldjoy)
        self.labvjoy = QLabel("%.2f" % self.ParamEmotionJoy, self)
        labnjoy = QLabel("Joy", self)
        sldjoylayout = QHBoxLayout()
        sldjoylayout.addWidget(labnjoy)
        sldjoylayout.addWidget(self.sldjoy)
        sldjoylayout.addWidget(self.labvjoy)

        # EmotionAngry
        self.sldang = QSlider(Qt.Horizontal, self)
        self.sldang.setRange(0, 100)    # 0.0 - 1.00
        self.sldang.setValue(0)         # 0.0
        self.sldang.valueChanged.connect(self.on_sldang)
        self.labvang = QLabel("%.2f" % self.ParamEmotionAngry, self)
        labnang = QLabel("Angry", self)
        sldanglayout = QHBoxLayout()
        sldanglayout.addWidget(labnang)
        sldanglayout.addWidget(self.sldang)
        sldanglayout.addWidget(self.labvang)

        # EmotionSad
        self.sldsad = QSlider(Qt.Horizontal, self)
        self.sldsad.setRange(0, 100)    # 0.0 - 1.00
        self.sldsad.setValue(0)         # 0.0
        self.sldsad.valueChanged.connect(self.on_sldsad)
        self.labvsad = QLabel("%.2f" % self.ParamEmotionSad, self)
        labnsad = QLabel("Sad", self)
        sldsadlayout = QHBoxLayout()
        sldsadlayout.addWidget(labnsad)
        sldsadlayout.addWidget(self.sldsad)
        sldsadlayout.addWidget(self.labvsad)

        # tts node speaker change
        btnspk = QPushButton('Change', self)
        btnspk.clicked.connect(self.on_click_spk)
        labnbtnspk = QLabel("Node speaker change", self)

        # tts node parameter change
        btnparam = QPushButton('Change', self)
        btnparam.clicked.connect(self.on_click_param)
        labnbtnparam = QLabel("Node param change", self)

        btnspklayout = QHBoxLayout()
        btnspklayout.addWidget(labnbtnspk)
        btnspklayout.addWidget(btnspk)
        btnspklayout.addWidget(labnbtnparam)
        btnspklayout.addWidget(btnparam)
        
        # get tts node's state
        btnnode = QPushButton('Import', self)
        btnnode.clicked.connect(self.on_click_node)
        labnbtnnode = QLabel("Import node state", self)

        # parameter display reset
        btnreset = QPushButton('Set', self)
        btnreset.clicked.connect(self.on_click_reset)
        labnbtnreset = QLabel("Default parameter", self)

        btnnodelayout = QHBoxLayout()
        btnnodelayout.addWidget(labnbtnnode)
        btnnodelayout.addWidget(btnnode)
        btnnodelayout.addWidget(labnbtnreset)
        btnnodelayout.addWidget(btnreset)
        
        sldlayout = QVBoxLayout()
        sldlayout.addLayout(combospklayout)
        sldlayout.addLayout(sldvollayout)
        sldlayout.addLayout(sldratelayout)
        sldlayout.addLayout(sldptlayout)
        sldlayout.addLayout(sldemlayout)
        sldlayout.addLayout(sldjoylayout)
        sldlayout.addLayout(sldanglayout)
        sldlayout.addLayout(sldsadlayout)
        sldlayout.addLayout(btnspklayout)
        sldlayout.addLayout(btnnodelayout)

        ttslayout = QVBoxLayout()
        ttslayout.addLayout(edtlayout)
        ttslayout.addLayout(btnlayout)
        ttslayout.addLayout(chklayout)

        wholelayout = QHBoxLayout()
        wholelayout.addLayout(ttslayout)
        wholelayout.addLayout(sldlayout)
        
        self.setLayout(wholelayout)

    # TTS開始
    def on_click_tts(self):
        ttstext = self.edttts.toPlainText()
        msg = {"Command" : "ttsPlaywithParamCB"}
        msg['TtsID'] = self.TTSID
        msg['Text'] = ttstext
        msg['Volume'] = self.ParamVolume
        msg['Rate'] = self.ParamRate
        msg['Pitch'] = self.ParamPitch
        msg['Emphasis'] = self.ParamEmphasis
        msgstyle = {}
        msgstyle['Joy'] = self.ParamEmotionJoy
        msgstyle['Angry'] = self.ParamEmotionAngry
        msgstyle['Sad'] = self.ParamEmotionSad
        msg['Emotion'] = msgstyle
        enc = json.dumps(msg)
        self.pub.publish(enc)
        self.TTSID += 1

    # volumeスライダの値変更
    def on_sldvol(self, value):
        self.ParamVolume = value / 100.0
        self.labvvol.setText("%.2f" % self.ParamVolume)

    # rateスライダの値変更
    def on_sldrate(self, value):
        self.ParamRate = value / 100.0
        self.labvrate.setText("%.2f" % self.ParamRate)

    # pitchスライダの値変更
    def on_sldpt(self, value):
        self.ParamPitch = value / 100.0
        self.labvpt.setText("%.2f" % self.ParamPitch)

    # emphasisスライダの値変更
    def on_sldem(self, value):
        self.ParamEmphasis = value / 100.0
        self.labvem.setText("%.2f" % self.ParamEmphasis)

    # emotion joyスライダの値変更
    def on_sldjoy(self, value):
        self.ParamEmotionJoy = value / 100.0
        self.labvjoy.setText("%.2f" % self.ParamEmotionJoy)

    # emotion angryスライダの値変更
    def on_sldang(self, value):
        self.ParamEmotionAngry = value / 100.0
        self.labvang.setText("%.2f" % self.ParamEmotionAngry)

    # emotion sadスライダの値変更
    def on_sldsad(self, value):
        self.ParamEmotionSad = value / 100.0
        self.labvsad.setText("%.2f" % self.ParamEmotionSad)

    # ドロップリストでspeakerを選択
    def on_speaker_changed(self, text):
        self.ParamVoiceName = text

    # 選択されているspeakerをtts nodeを指定する
    def on_click_spk(self):
        msg = {"Command" : "ttsSpeakerChange"}
        msg['Speaker'] = self.ParamVoiceName
        enc = json.dumps(msg)
        self.pub.publish(enc)

    # 画面のパラメータを初期値にリセット
    def on_click_reset(self):
        self.ParamVolume = 1.0
        self.ParamRate = 1.0
        self.ParamPitch = 1.0
        self.ParamEmphasis = 1.0
        self.ParamEmotionJoy = 0.0
        self.ParamEmotionAngry = 0.0
        self.ParamEmotionSad = 0.0
        self.sldvol.setValue(100)     # 1.00
        self.sldrate.setValue(100)    # 1.00
        self.sldpt.setValue(100)      # 1.00
        self.sldem.setValue(100)      # 1.00
        self.sldjoy.setValue(0)       # 0.0
        self.sldang.setValue(0)       # 0.0
        self.sldsad.setValue(0)       # 0.0

    # TTSおよび再生停止
    def on_click_stop(self):
        msg = {"Command": "audioStop"}
        enc = json.dumps(msg)
        self.pub.publish(enc)

    # TTS nodeの状態を取り込む
    def on_click_node(self):
        msg = {"Command": "ttsSystemParam"}
        enc = json.dumps(msg)
        self.pub.publish(enc)

    # TTS nodeにspeaker parameterを送る
    def on_click_param(self):
        msg = {"Command": "ttsSpeakerParamChange"}
        msg['Volume'] = self.ParamVolume
        msg['Rate'] = self.ParamRate
        msg['Pitch'] = self.ParamPitch
        msg['Emphasis'] = self.ParamEmphasis
        msgstyle = {}
        msgstyle['Joy'] = self.ParamEmotionJoy
        msgstyle['Angry'] = self.ParamEmotionAngry
        msgstyle['Sad'] = self.ParamEmotionSad
        msg['Emotion'] = msgstyle
        enc = json.dumps(msg)
        self.pub.publish(enc)
         
    # 本プログラムを終了
    def on_click_close(self):
        self.close()

    # Carlos AudioPlayerのON/OFF
    def on_check_cplayer(self, state):
        if state == Qt.Checked:
            self.ExternalAudioPlayerFlag = 1
            msg = {"Command": "ttsOutput", "Flag": 1}
        else:
            self.ExternalAudioPlayerFlag = 0
            msg = {"Command": "ttsOutput", "Flag": 0}
        enc = json.dumps(msg)
        self.pub.publish(enc)

    # Local AudioPlayerのON/OFF
    def on_check_lplayer(self, state):
        if state == Qt.Checked:
            self.LocalAudioPlayerFlag = 1
            msg = {"Command": "ttsLocalOutput", "Flag": 1}
        else:
            self.LocalAudioPlayerFlag = 0
            msg = {"Command": "ttsLocalOutput", "Flag": 0}
        enc = json.dumps(msg)
        self.pub.publish(enc)

    # ros subscriberのcallback関数からQPlainTextEditに描画するための関数
    def on_edit_text(self):
        self.edtrcv.setPlainText(json.dumps(self.recv_dict, indent=2))
x
    # ros subscriberのcallback関数からQPlainTextEditに描画するための関数
    def on_widget_update(self):
        # スライダで設定するパラメータの更新
        self.labvvol.setText("%.2f" % self.ParamVolume)
        self.labvrate.setText("%.2f" % self.ParamRate)
        self.labvpt.setText("%.2f" % self.ParamPitch)
        self.labvem.setText("%.2f" % self.ParamEmphasis)
        self.labvjoy.setText("%.2f" % self.ParamEmotionJoy)
        self.labvang.setText("%.2f" % self.ParamEmotionAngry)
        self.labvsad.setText("%.2f" % self.ParamEmotionSad)
        self.sldvol.setValue(int(self.ParamVolume*100))
        self.sldrate.setValue(int(self.ParamRate*100))
        self.sldpt.setValue(int(self.ParamPitch*100))
        self.sldem.setValue(int(self.ParamEmphasis*100))
        self.sldjoy.setValue(int(self.ParamEmotionJoy*100))
        self.sldang.setValue(int(self.ParamEmotionAngry*100))
        self.sldsad.setValue(int(self.ParamEmotionSad*100))
        # チェックボックスで設定するパラメータの更新
        if self.ExternalAudioPlayerFlag == 0:
            if self.chkcplayer.isChecked():
                self.chkcplayer.toggle()
        elif not self.chkcplayer.isChecked():
            self.chkcplayer.toggle()

        if self.LocalAudioPlayerFlag == 0:
            if self.chklplayer.isChecked():
                self.chklplayer.toggle()
        elif not self.chklplayer.isChecked():
            self.chklplayer.toggle()

        # ドロップダウンリストで設定するパラメータの更新
        index = self.combospk.findText(self.ParamVoiceName)
        if index >= 0:
            self.combospk.setCurrentIndex(index)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())
