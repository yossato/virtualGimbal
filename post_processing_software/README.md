# Calibration
Calibration is necessary before executing movie stabilization. By executing the calibration, you can acquire the focal length of the lens, internal parameters such as image center, and lens distortion coefficient.

## Taking a video of a chess board pattern for calibration
Before activating the calibration tool, we will explain the shooting procedure of the movie data necessary for the calibration.
Open the [PDF](https://github.com/yossato/macadamia/raw/master/post_processing_software/doc/chessboardpattern.pdf) of the chessboard with PC or tablet and display it in full screen.  Turn off camera shake compensation on the camera, then shoot movies on the screen with the camera.  
A sample chess board video can be found at [https://drive.google.com/open?id=18N9hlZ-U4sec1Opt3HHLgL9xHEMN3SoX](https://drive.google.com/open?id=18N9hlZ-U4sec1Opt3HHLgL9xHEMN3SoX). Move the camera so that the chess board appears at various distances and angles during shooting.  
After shooting, copy the movie to the appropriate directory on the PC. The following explanation assumes that the movie is saved in the following directory:
`~/vgdataset/calibration_ILCE-6500_SEL1670Z_4K.MP4  `  
Turn ON camera shake correction.

## Parameter estimation by calibration tool
Internal parameters can be acquired with the camera_calibration tool.
Please select the direction of the SD card slot at calibration by selecting a number from the following images.
![SD Card slot direction](https://github.com/yossato/macadamia/raw/master/post_processing_software/doc/sd_card_rotation.png "SD Card slod direction")　

Launch camera_calibration with the following command. It may take up to 5 minutes to complete.
`$ cd ~/virtualGimbal/build`   
`$ ./camera_calibration -i ~/vgdataset/calibration_ILCE-6500_SEL1670Z_4K.MP4 -c ILCE-6500 -l SEL1670Z -r 4`
### Explanation of options
-'i' option is the movie file name of the chessboard pattern.  
-'c' option is the name of the camera. Here, we showed an example using ILCE-6500 of SONY's DLSR.  
-'l' option is the name of the lens. Here we showed an example using the SONY zoom lens, SEL 1670Z.  
-'r' option inputs the direction of the SD card slot selected from the above image with a number. In this case, I chose the direction No.4 that inserts the SD card to the Camera from the bottom of it while the notch is in the back. Since the direction of this insertion is different for each camera, please choose a number for each camera. All options are mandatory.

When you start the camera_calbration tool, it executes chess board detection and estimates internal parameters. The results will be recorded in camera_descriptions/cameras.json. cameras.json is a file that records camera information in JSON format.

## Initial setting
Execute the following command __only once__ to complete the initial setting of your PC. By this procedure, you can fix VirtualGimbal's device file name to /dev/ttyVIG0.  
`$ sudo ./scripts/setup_udev_rules.sh`

# Erase built-in flash memory
VirtualGimbal has built-in flash memory for recording camera motion separately from microSD card. It is necessary to erase this flash memory before shooting a movie. Please note that the camera motion data will be erased.
You can erase the Flash Memory of the VirtualGimbal by the following command.  
`$ ./scripts/erase_flash_memory.py`

# Shooting movies with VirtualGimbal
Set the microSD card in VirtualGimbal first. Next, please put the virtual Gimbal into the SD card slot of the camera the same way as for a standard SD card. In order to prevent interference with virtual Gimbal's software stabilization correction, please turn off the camera's built-in camera shake correction function before shooting. For cameras that can set the shutter speed of the camera, please shorten the shutter speed as quickly as possible. The blur of the image decreases and the image becomes more evident when the shutter speed is quicker. Approximately 1/300 second is a standard. After performing the above procedure, turn on the camera and take the movie. If VirtualGimbal is not recognized by the camera properly, please try inserting/removing the microSD card and VirtualGimbal.

I will assume that you have saved the captured video as  
 `~/vgdataset/myfirstvideo.MP4`.  
After movie shooting is completed, it is good to turn the camera shake correction function back on so that you do not have to worry about shooting pictures and videos later.

# Stabilizing videos with VirtualGimbal
A post-processing software stabilizes your video.

## Extract angular velocity information from VirtualGimbal
I will explain how to retrieve camera motion information recorded in VirtualGimbal. The information to be extracted is the angular velocity of the camera. You can obtain angular velocity data of JSON format from VirtualGimbal via USB.
Remove the microSD card from VirtualGimbal and connect it to the USB port of the PC via the adapter as shown below. Do not insert VirtualGimbal into the SD card slot of PC at same time.  
![Connection via USB](https://github.com/yossato/macadamia/raw/master/post_processing_software/doc/connect_via_usb.jpg "Connection via USB")  
![Connector](https://github.com/yossato/macadamia/raw/master/post_processing_software/doc/connector.jpg "Connector")  
The angular velocity data of JSON format can be acquired with the following command.  
`$ ./scripts/generate_angular_velocity_json.py`

Please use the JSON filename of the angular velocity displayed on the screen at stabilization processing.  
In the following explanation, it is assumed that the JSON file name is as follows.  
`records/2019-02-16_18.35.24.json`

## Movie stabilization by post-processing stabilization tool VirtualGimbal
Execute the following command to stabilize the movie. While processing, a comparison between the original video and the stabilized video is displayed.  
`$ ./virtualGimbal -i ~/vgdataset/myfirstvideo.MP4 -j records/2019-02-16_18.35.24.json -z 1.3  -c ILCE-6500 -l SEL1670Z`

Stabilized videos are saved below.  
`~/vgdataset/myfirstvideo.MP4_deblured.avi`

### Explanation of options
-i specifies the filename of the input movie.  
-j specifies the JSON file that recorded the angular velocity.  
-z specifies the zoom magnification. This value should be around 1.1 to 1.5. Increasing the numerical value will improve the stabilizing ability but the information at the edge of the screen will be lost.  
-c specifies the camera name  
-l specifies the lens name  
-o is specified when saving the stabilization result as a movie.  

# Japanese language

# キャリブレーション
動画安定化を実行する前にキャリブレーションが必要です。キャリブレーションを実行することでレンズの焦点距離、画像中心などの内部パラメータ、及びレンズ歪係数を取得できます。

## チェスボードパターンの動画を撮影
キャリブレーションツールを起動する前に、キャリブレーションに必要な動画データの撮影手順を説明します。
PCやタブレットでチェスボードのPDF(doc/chessboardpattern.pdf)を開き、全画面表示します。
カメラの手ブレ補正をOFFにした後、画面の動画をカメラで撮影します。
撮影中に様々な距離と角度でチェスボードが映るようにカメラを動かします。
撮影が終わったらPCの適当なディレクトリへ動画をコピーしてください。
以降の説明は動画を下記のディレクトリに保存したと仮定して説明します。

`~/vgdataset/calibration_ILCE-6500_SEL1670Z_4K.MP4`  

動画のサンプルは[https://drive.google.com/open?id=18N9hlZ-U4sec1Opt3HHLgL9xHEMN3SoX](https://drive.google.com/open?id=18N9hlZ-U4sec1Opt3HHLgL9xHEMN3SoX)で見ることができます。
カメラの手ブレ補正はONに戻してください。

## キャリブレーションツールによるパラメータ推定
内部パラメータはcamera_calibrationツールで取得できます。  
SDカードスロットの向きは以下の画像から数字を選んでキャリブレーション時に指定してください。  
![SD Card slot direction](https://github.com/yossato/macadamia/raw/master/post_processing_software/doc/sd_card_rotation.png "SD Card slod direction")　　

次のコマンドでcamera_calibrationを起動します。完了まで5分程度かかる可能性があります。   
`$ cd ~/virtualGimbal/build`   
`$ ./camera_calibration -i ~/vgdataset/calibration_ILCE-6500_SEL1670Z_4K.MP4 -c ILCE-6500 -l SEL1670Z -r 4`

-i オプションはチェスボードパターンの動画のファイル名です。  
-c オプションはカメラの名前です。ここではSONYのDLSRのILCE-6500を使用した場合の例を示しました。  
-l オプションはレンズの名前です。ここではSONYのズームレンズ、SEL1670Zを用いた例を示しました。  
-r オプションは上の画像の中から選んだSDカードスロットの向きを数字で入力します。ここでは4番のカメラの下方からSDカードの切り欠きを手前に向けて挿入する向きを選びました。カメラごとにこの挿入の向きは異なるので、カメラごとに数字を選んでください。

全てのオプションは必須項目です。

camera_calbrationツールを起動するとチェスボードの検出を実行し内部パラメータを推定します。結果はcamera_descriptions/cameras.jsonに記録されます。cameras.jsonはJSON形式でカメラ情報を記録するファイルです。  

## 初期設定
一度だけ以下のコマンドを実行しVirtual COM Portの初期設定を完了させてください。この手順でデバイスファイル名を/dev/ttyVIG0に固定することができます。
`$ sudo ./scripts/setup_udev_rules.sh`

# VirtualGimbal 内蔵 Flash memory の消去
VirtualGimbalはmicroSDカードとは別にカメラの動きを記録するためのFlash memoryを内蔵しています。動画を撮影する前にこのFlash memoryを消去する必要があります。初期化により以前に記録されたカメラの動きのの情報は消去されることに注意してください。
以下の手順によりVirtualGimbal内臓のFlash Memoryを消去できます。  
`$ ./scripts/erase_flash_memory.py`

# VirtualGimbal を使った動画の撮影
VirtualGimbalをカメラに挿入して動画を撮影する手順を説明します。
まずVirtualGimbalにmicroSDカードへセットしてください。次に通常のSDカードを同様に、VirtualGimbalをカメラのSDカードスロットへセットしてください。VirtualGimbalのソフトウェア安定化補正と干渉するのを防ぐために、撮影前にカメラの内蔵の手ブレ補正機能はOFFにしてください。
カメラのシャッタースピードを設定できるカメラでは、なるべくシャッタースピードを早く、短くしてください。画像のブラーが減少して画像が鮮明になります。1/300 second 程度が目安です。
以上の手順を実行後、カメラの電源をいれてあなたの動画を撮影してください。
VirtualGimbalが正常にカメラに認識されない場合は、microSDカードおよびVirtualGimbalの抜き差しを試してください。

以降、撮影した動画を`~/vgdataset/myfirstvideo.MP4`として保存したと仮定して説明します。
動画撮影が完了した後は、あなたが後で写真や動画を撮影するときに困らないように、カメラの手ブレ補正機能をONに戻しておくと良いです。

# VirtualGimbal を使った動画の安定化
動画をPCの後処理で安定化させます。

## VirtualGimbal から角速度情報を抜き出す
VirtualGimbalに記録されたカメラの動きの情報を取り出す方法を説明します。取り出す情報はカメラの角速度です。USB経由でVirtualGimbalからJSON形式の角速度データを取得できます。
VirtualGimbalからmicroSDカードを取り出し以下の図のとおりにアダプタを介してPCのUSBポートへ接続してください。この時、VirtualGimbalをPCのSDカードスロットへ同時に挿入しないでください。  
![Connection via USB](https://github.com/yossato/macadamia/raw/master/post_processing_software/doc/connect_via_usb.jpg "Connection via USB")
![Connector](https://github.com/yossato/macadamia/raw/master/post_processing_software/doc/connector.jpg "Connector")  

以下のコマンドによりJSON形式の角速度データを取得できます。
`$ ./scripts/generate_angular_velocity_json.py`

画面に表示された角速度のJSONファイル名は、安定化処理の時に使用してください。
以降の説明ではJSONファイル名が以下の場合だったとして説明します。
`records/2019-02-16_18.35.24.json`

## 事後処理安定化ツール VirtualGimbalによる動画安定化
以下のコマンドを実行して動画を安定化させます。処理している間、オリジナル動画と安定化後の動画の比較が表示されます。  
`$ ./virtualGimbal -i ~/vgdataset/myfirstvideo.MP4 -j records/2019-02-16_18.35.24.json -z 1.3  -c ILCE-6500 -l SEL1670Z`


安定化された動画は`~/vgdataset/myfirstvideo.MP4_deblured.avi`として保存されます。

### オプションの説明
-i は入力動画のファイル名を指定します。  
-j は角速度を記録したJSONファイルを指定します。  
-z はズーム倍率を指定します。この値は1.1から1.5程度が良いでしょう。数値を大きくすると安定化能力が向上しますが画面の端の情報が失われます。  
-c はカメラ名を指定します  
-l はレンズ名を指定します  
-o は安定化結果を動画として保存する場合に指定します。  
