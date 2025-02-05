#ifndef _TTSPROTOCOL_H_
#define _TTSPROTOCOL_H_

// 送信用プロトコル
// どの命令も、JSON形式で記述したものを1つのstring変数に格納して送る
// 日本語textはutf-8
/////// 音声再生を一時停止する /////// 
#define TTS_AUDIO_PUASE "audioPause"
// example
//{
//  "Command": "audioPause"
//}

/////// 音声再生を再開する /////// 
#define TTS_AUDIO_RESUME "audioResume"
// example
//{
//  "Command": "audioResume"
//}

/////// 音声再生を停止する ///////
#define TTS_AUDIO_STOP "audioStop"
// example
//{
//  "Command": "audioStop"
//}

///////  音声再生プロセスの状態を得る /////// 
#define TTS_AUDIO_ASK_STATUS "audioAskStatus"  
// example
//{
//  "Command": "audioAskStatus"
//}
// example of return
//{
//  "Message": "PlayStatusInPlay"
//}

///////  指定した音声ファイルを再生する ///////  
#define TTS_AUDIO_PLAY_WAV "audioPlayWavefile"
// 音声ファイル読み込みに成功すればACK、失敗すればNAKが返ってくる
// example 
//{
//  "Command": "audioPlayWavefile",
//  "Wavefile": "hello.wav"
//}
// example of return
//{
//  "Message": "WavefileLoadACK",
//  "Wavefile": "hello.wav"
//}

///////  指定した音声ファイルを再生する（再生終了時のコールバック要求）///////  
#define TTS_AUDIO_PLAY_WAV_CB "audioPlayWavefileCB"
// 音声ファイル読み込みに成功すればACK、失敗すればNAKが返ってくる
// 再生が終了するとコールバックが返ってくる
// example 
//{
//  "Command": "audioPlayWavefileCB",
//  "Wavefile": "hello.wav"
//}
// example of return
//読込ができたかについて
//{
//  "Message": "WavefileLoadACK",
//  "Wavefile": "hello.wav"
//}
// 再生が終了した際に
//{
//  "Message": "PlayWavefileFinish",
//  "Wavefile: "hello.wav"
//}

/////// 音声合成の話者を変更する ///////  
#define TTS_SPEAKER_CHANGE "ttsSpeakerChange"
// 話者名を指定する
// 変更できたらACK、変更がなければNAKが返ってくる
// example 
//{
//  "Command": "ttsSpeakerChange",
//  "Speaker": "maki_emo_22_standard"
//}
// example of return
//読込ができたかについて
//{
//  "Message": "SpeakerChangeNAK"
//}

/////// 音声合成して再生する ///////
#define TTS_SYNTH_PLAY "ttsPlay"
// テキストを指定する
// TtsIDにuserが決めたIDを指定する
// 前に送ったテキストが再生中や一時停止中のときは、送っても無視される
// 送る前にTTS_AUDIO_ASK_STATUSコマンドで再生が停止していることを確認してから送ること
// example
//{
//  "Command": "ttsPlay",
//  "TtsID": 1023,
//  "Text": "おはようございます。"
//}

/////// 音声合成して再生する ///////
#define TTS_SYNTH_PLAY_PARAM "ttsPlaywithParam"
// テキストを指定する
// 合成パラメータも指定する
// TtsIDにuserが決めたIDを指定する
// withParamの場合は合成パラメータも指定する
// 前に送ったテキストが再生中や一時停止中のときは、送っても無視される
// 送る前にTTS_AUDIO_ASK_STATUSコマンドで再生が停止していることを確認してから送ること
// example
//{
//  "Command": "ttsPlaywithParam",
//  "TtsID": 1023,
//  "Text": "おはようございます。",
//  "Volume": 1.3,
//  "Rate": 1.0,
//  "Pitch": 1.2,
//  "Emphasis": 1.0,
//  "Emotion":
//  {
//    "Joy": 1.0,
//    "Angry": 0.0,
//    "Sad": 0.0
//  }
//}

/////// 音声合成して再生する（再生終了時のコールバック要求） ///////
#define TTS_SYNTH_PLAY_CB "ttsPlayCB"
// テキストを指定する
// TtsIDにuserが決めたIDを指定する
// 前に送ったテキストが再生中や一時停止中のときは、送っても無視される
// 送る前にTTS_AUDIO_ASK_STATUSコマンドで再生が停止していることを確認してから送ること
// 再生が終了するとコールバックが返ってくる
// example
//{
//  "Command": "ttsPlayCB",
//  "TtsID": 1023,
//  "Text": "おはようございます。"
//}
//
// example of return
//{
//  "Message": "PlayTTSVoiceFinish",
//  "TtsID": 1023
//}

/////// 音声合成して再生する（再生終了時のコールバック要求） ///////
#define TTS_SYNTH_PLAY_PARAM_CB "ttsPlaywithParamCB"
// テキストを指定する
// 合成パラメータも指定する
// TtsIDにuserが決めたIDを指定する
// withParamの場合は合成パラメータも指定する
// 前に送ったテキストが再生中や一時停止中のときは、送っても無視される
// 送る前にTTS_AUDIO_ASK_STATUSコマンドで再生が停止していることを確認してから送ること
// 再生が終了するとコールバックが返ってくる
// example
//{
//  "Command": "ttsPlaywithParamCB",
//  "TtsID": 1023,
//  "Text": "おはようございます。",
//  "Volume": 1.3,
//  "Rate": 1.0,
//  "Pitch": 1.2,
//  "Emphasis": 1.0,
//  "Emotion":
//  {
//    "Joy": 1.0,
//    "Angry": 0.0,
//    "Sad": 0.0
//  }
//}
// example of return
//{
//  "Message": "PlayTTSVoiceFinish",
//  "TtsID": 1023
//}

/////// 音声合成してwavefileとして保存する ///////
#define TTS_SYNTH_WAV_SAVE "ttsWaveSave"
// 音声合成してwavefileとして保存する
// 再生が終了するとコールバックが返ってくる
// TtsIDにuserが決めたIDを指定する
// 保存したファイル名が返ってくる
// 音声合成してwavefileとして保存する
//{
//  "Command": "ttsWaveSave",
//  "TtsID": 123,
//  "Text": "おはようございます。"
//}
// example of return
// 生成したファイル名
//{
//  "Message": "WavefileName",
//  "Wavefile": "2021_02_11_13_23.wav"
//}
// 再生が終了した際に
//{
//  "Message": "PlayTTSVoiceFinish",
//  "TtsID": 123
//}

/////// 音声合成してwavefileとして保存する ///////
#define TTS_SYNTH_WAV_PARAM_SAVE "ttsWavewithParamSave"
// withParamの場合は合成パラメータも指定する
// TtsIDにuserが決めたIDを指定する
// 再生が終了するとコールバックが返ってくる
// 保存したファイル名が返ってくる
// 音声合成してwavefileとして保存する
//{
//  "Command": "ttsWaveSave",
//  "Text": "おはようございます。"
//  "TtsID": 321,
//  "Text": "おはようございます。",
//  "Volume": 1.3,
//  "Rate": 1.0,
//  "Pitch": 1.2,
//  "Emphasis": 1.0,
//  "Emotion":
//  {
//    "Joy": 1.0,
//    "Angry": 0.0,
//    "Sad": 0.0
//  }
//}
// example of return
// 生成したファイル名
//{
//  "Message": "WavefileName",
//  "Wavefile": "2021_02_11_13_23.wav"
//}
// 再生が終了した際に
//{
//  "Message": "PlayTTSVoiceFinish",
//  "TtsID": 321
//}

/////// 音声合成の話者のパラメータを変更する ///////
#define TTS_SYNTH_SETSPKPARAM "ttsSpeakerParamChange"
// 変更できるのは
// 音量（Volume）float
// 話速（Rate）float
// ピッチ（Pitch）float
// 抑揚（Emphasis）float
// 感情（Joy）float, （Angry）float, （Sad）float
// 変更できたらACK、変更がなければNAKが返ってくる
//{
//  "Command": "ttsSpeakerParamChange",
//  "Volume": 1.3,
//  "Rate": 1.0,
//  "Pitch": 1.2,
//  "Emphasis": 1.0,
//  "Emotion":
//  {
//    "Joy": 1.0,
//    "Angry": 0.0,
//    "Sad": 0.0
//  }
//}
// example of return
//{
//  "Message": "SpeakerParamChangeACK"
//}

/////// TTSがBusy状態かIdle状態かを得る ///////
#define TTS_ASK_STATUS "ttsStatus"
//{
//  "Command": "ttsStatus"
//}
// example of return
// BUSY か IDLEが返ってくる
//{
//  "Message": "TTSStatusBusy"
//}

/////// Carlos AudioPlayerにdataを送る ///////
#define TTS_OUTPUT_SELECT "ttsOutput"
// Flagが1ならCarlos AudioPlayerにdataを送る．0なら送らない．
//{
//  "Command": "ttsOutput",
//  "Flag": 1
//}

/////// Local AudioPlayerを使用する ///////
#define TTS_LOCALOUTPUT_SELECT "ttsLocalOutput"
// Flagが1ならローカルPCで音声を再生する．0なら再生しない．
//{
//  "Command": "ttsLocalOutput",
//  "Flag": 1
//}

/////// TTS nodeのシステムの現在のパラメータを得る ///////
#define TTS_ASK_SYSTEMPARAM "ttsSystemParam"
//{
//  "Command": "ttsSystemParram"
//}
// example of return
// 現在の話者や話者のパラメータ等が返ってくる
//{
//  "Message": "SystemParam",
//  "Speaker": "maki_emo_22_standard",
//  "Volume": 1.3,
//  "Rate": 1.0,
//  "Pitch": 1.2,
//  "Emphasis": 1.0,
//  "Emotion":
//  {
//    "Joy": 1.0,
//    "Angry": 0.0,
//    "Sad": 0.0
//  },
//  "LocalAudioPlayerFlag": 1,
//  "ExternalAudioPlayerFlag": 0
//}

#endif
