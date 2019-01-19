# 2018fall---BLE-mouse-with-handwriting-recognition(正式報告請參閱pdf檔)
# 一.	動機
   熟悉較為底層的系統程式及驅動程式以完成BLE傳輸、三軸加速度計讀值、GPIO操控、虛擬滑鼠及文字輸入等功能，搭配openCV之套件抓取手寫軌跡並辨識為何字母。
   此作品盡可能使用便宜的器材來體現工業生產上的成本考量，大致上需要CC26x2、ADXL345、raspberry pi，其中運行於raspberry pi的程式能夠移植至其他裝有linux作業系統的機器。
   另外此作品也盡量落實嵌入式裝置能量節省的原則，程式中以一些特殊的作法來達到此目的。
# 二. 開發環境
* TI CC26X2
* Raspberry Pi 3
* ADXL345
* python3.5
* openCV 3.1.0
* Tesseract-OCR 4.00

# 三. 作法
## TI CC26X2
* I2C操作ADXL345
* GPIO感應按鍵
* 發送BLE廣播

## Raspberry Pi 3
* BLE封包解讀及虛擬滑鼠操控
* openCV抓取軌跡及Tesseract辨識字母
* Input subsystem驅動程式控制滑鼠及鍵盤

# 四. 成果
* 滑鼠游標可向上下、左右、斜向大致順暢移動。並可依CC26X2傾斜程度有不同速度及斜率。
* 滑鼠左鍵、右鍵、兩鍵齊按可正常使用。
* 滑鼠按鍵搭配游標移動可做出拖曳、滾輪等功能。
* 滑鼠長按左鍵開啟寫字功能，可直接在空中書寫。

# 五. How to reproduce
1. Clone all the repo
2. 開啟滑鼠驅動程式
```
cd /subsystem_driver
make
sudo insmod virmouse.ko
```
3. 執行ble mouse
```
cd /ble_and_mouse
make
sudo ./mouse &
```
4. 開啟寫字功能
Install OpenCV and Tesseract-OCR 
```
cd /color_trace
sudo python3 object_movement.py
```
# 六. Reference
1.	https://www.pyimagesearch.com
2.  https://github.com/tesseract-ocr/tesseract/wiki/TrainingTesseract-4.00
