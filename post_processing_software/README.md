# キャリブレーション
動画安定化を実行する前にキャリブレーションが必要です。キャリブレーションを実行することでレンズの焦点距離、画像中心などの内部パラメータ、及びレンズ歪係数を取得できます。

## チェスボードパターンの動画を撮影
キャリブレーションツールを起動する前に、キャリブレーションに必要な動画データの撮影手順を説明します。
PCやタブレットでチェスボードのPDC(doc/chessboardpattern.pdf)を開き、全画面表示します。
カメラの手ブレ補正をOFFにした後、画面の動画をカメラで撮影します。
撮影中に様々な距離と角度でチェスボードが映るようにカメラを動かします。
撮影が終わったらPCの適当なディレクトリへ動画をコピーしてください。以降~/vgdataset/calibration_ILCE-6500_SEL1670Z_4K.MP4として動画を保存したと仮定して説明します。
カメラの手ブレ補正はONに戻してください。
動画のサンプルは＊＊＊で見ることができます。

## キャリブレーションツールによるパラメータ推定
内部パラメータはcamera_calibrationツールで取得できます。  
SDカードスロットの向きは以下の画像から数字を選んでキャリブレーション時に指定してください。  
![SD Card slot direction](https://github.com/yossato/macadamia/raw/master/post_processing_software/doc/sd_card_rotation.png "SD Card slod direction")　　
  
次のコマンドでcamera_calibrationを起動します。完了まで5分程度かかる可能性があります。  
$ ./camera_calibration -i ~/vgdataset/calibration_ILCE-6500_SEL1670Z_4K.MP4 -c ILCE-6500 -l SEL1670Z -r 4

-i オプションはチェスボードパターンの動画のファイル名です。  
-c オプションはカメラの名前です。ここではSONYのDLSRのILCE-6500を使用した場合の例を示しました。  
-l オプションはレンズの名前です。ここではSONYのズームレンズ、SEL1670Zを用いた例を示しました。  
-r オプションは上の画像の中から選んだSDカードスロットの向きを数字で入力します。ここでは4番のカメラの下方からSDカードの切り欠きを手前に向けて挿入する向きを選びました。カメラごとにこの挿入の向きは異なるので、カメラごとに数字を選んでください。

全てのオプションは必須項目です。

camera_calbrationツールを起動するとチェスボードの検出を実行し内部パラメータを推定します。結果はcamera_descriptions/cameras.jsonに記録されます。cameras.jsonはJSON形式でカメラ情報を記録するファイルです。  

## 初期設定
一度だけ以下のコマンドを実行しVirtual COM Portの初期設定を完了させてください。
$ sudo ./scripts/setup_udev_rules.sh

# virtualGimbal を使った動画の撮影
virtualGimbalをカメラに挿入して動画を撮影する手順を説明します。
動画はvirtualGimbalにmicroSDカードをセットして、virtualGimbalをカメラにセットして撮影します。virtualGimbalのソフトウェア安定化補正と干渉するため、撮影前にカメラの内蔵の手ブレ補正機能はOFFにしてください。
カメラのシャッタースピードを設定できるカメラでは、なるべくシャッタースピードを早く、短くしてください。画像のブラーが減少して画像が鮮明になります。1/300 second 程度が目安です。
以降、撮影した動画を~/vgdataset/myfirstvideo.MP4として保存したと仮定して説明します。
動画撮影が完了した後は、あなたが後で写真や動画を撮影するときに困らないように、カメラの手ブレ補正機能をONに戻しておくと良いです。

# virtualGimbal を使った動画の安定化
動画をPCの後処理で安定化させます。

## virtualGimbal から角速度情報を抜き出す
virtualGimbalに記録されたカメラの動きの情報を取り出す方法を説明します。取り出す情報はカメラの角速度です。USB経由でvirtualGimbalからJSON形式の角速度データを取得できます。
virtualGimbalからmicroSDカードを取り出し以下の図のとおりにアダプタを介してPCのUSBポートへ接続してください。この時、virtualGimbalをPCのSDカードスロットへ挿入しないでください。

以下のコマンドによりJSON形式の角速度データを取得できます。
$ ./scripts/generate_angular_velocity_json.py

画面に表示された角速度のJSONファイル名は、安定化処理の時に使用してください。
以降の説明ではJSONファイル名が以下の場合だったとして説明します。
records/2019-02-16_18.35.24.json

## 事後処理安定化ツール virtualGimbalによる動画安定化
以下のコマンドを実行して動画を安定化させます。処理している間、オリジナル動画と安定化後の動画の比較が表示されます。  
  
./virtualGimbal -i ~/vgdataset/myfirstvideo.MP4 -j records/2019-02-16_18.35.24.json -z 1.3  -c ILCE-6500 -l SEL1670Z -o  
  
安定化された動画は~/vgdataset/myfirstvideo.MP4_deblured.aviとして保存されます。

### オプションの説明
-i は入力動画のファイル名を指定します。
-j は角速度を記録したJSONファイルを指定します。
-z はズーム倍率を指定します。この値は1.1から1.5程度が良いでしょう。数値を大きくすると安定化能力が向上しますが画面の端の情報が失われます。
-c はカメラ名を指定します
-l はレンズ名を指定します
-o は安定化結果を動画として保存する場合に指定します。
