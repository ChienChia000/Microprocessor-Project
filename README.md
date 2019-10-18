# Microprocessor-Project-穿戴式步頻訓練帶
組員：何鑑家、施彥安、蔡明修

- 初審報告：https://hackmd.io/hZmRrTMcSMGqQZN4KTT9YA
- 會議記錄：https://hackmd.io/XXZPixIeQf-v-4AEFZlKbQ?view
- DEMO影片：https://www.youtube.com/watch?v=tSb-jIi52nY

## 專題動機與願景
- 在田徑項目當中，不論是短距離、中距離還是長距離。在間歇跑訓練時，適當的控制 步頻是相當重要的。因為步頻的大小會直接影響速度。
- 所以此專題旨在設計穿戴式裝置，讓運動員在運動時可以即時了解自己的步頻，並可 透過預設的步頻範圍來提醒自己運動當下所需調整的節奏。

## 系統功能與原理說明
1. 系統功能
    - 使用者可以透過三個光敏電阻所組成的設定面板，依據滑動方向的手勢來調整七段顯 示器上的步頻範圍，設定完畢後用手蓋住此面板即可進入運動狀態。
    - 運動狀態時七段顯示器所顯示的值為跑步當下的步頻，並每四秒更新一次。
    - 當步頻處於正常範圍時，三顆可變色	ＬＥＤ燈會顯示<font color="#00dd00">**綠色**</font><br />，呼吸燈會提供建議的呼吸 頻率，而蜂鳴器不會發出警示聲響。
    - 當步頻過快時，ＬＥＤ燈會顯示<font color="#dd0000">**紅色**</font><br />，呼吸燈建議的頻率會加快，而蜂鳴器會發出緩 慢的警示聲響提醒使用者放慢速度。
    -  當步頻過慢時，ＬＥＤ燈會顯示<font color="#0000dd">**藍色**</font><br />，呼吸燈建議的頻率會減慢，而蜂鳴器會發出急 促的警示聲響提醒使用者加快速度。
3. 原理說明
    - 運用 3 個光敏電阻排成陣列，透過 ADC 將類比轉成數數位數值，並依照 3 個光敏電阻 數值變化的順序，判斷手勢為為往左或往右。
    - 由於擺臂次數與步伐相同，因此運用超音波模組測量手腕到身體的距離來判斷是否擺 臂一次，以計算步伐次數。

## 創意特色描述
1.  輕量化
    - 測量步頻的裝置不是戴在腳上，而是位於手腕上，透過偵測擺臂的方式，推 算步頻，如此可減輕運動員步伐的負擔及意外發生機率。
2.  校正運動員運動員擺臂姿勢
    - 由於標準的擺手姿勢為手腕從肩膀高度下擺置身體後 面，再由後方擺置肩膀高度。因此，步頻的偵測透過擺手過身體的次數來計算，以督 促運動員以正確的擺手練習。
3.  人性化操作
    - 運用三個光敏電阻取代傳統按鈕，設定時不需觸碰到面板，僅透過手勢 滑動即可調整數值，使用上使用上更加簡便和人性化。	

## 系統使用環境及對象
- 適合各種距離的田徑運動員
- 在有光線的環境下皆可使用（如太陽光、路燈......等）

## 系統完整架構圖、流程圖、電路圖、設計圖
- 系統架構圖
![](https://i.imgur.com/QfXGJrM.png)
- 流程圖
![](https://i.imgur.com/BgrF7BP.png)
- 電路圖
![](https://i.imgur.com/4RHaiKJ.png)
- 設計圖
    - 第一層
    ![](https://i.imgur.com/WksQ2jZ.png)
    - 第二層
    ![](https://i.imgur.com/Wy1A93g.png)
    - 第三層
    ![](https://i.imgur.com/llbXR90.png)

## 系統開發工具、材料、技術
- 系統開發工具：MPLAB
- 材料：
    - PIC18F4520
    - pickit3
    - 光敏電阻 X 3
    - 七段顯示器 X 1
    - 超音波模組	X 1
    - ＲＧＢ ＬＥＤ燈 X 3
    - 藍色ＬＥＤ燈 X 1
    - 蜂鳴器 X 1
    - 開關 X 1
    - 行動電源 X 1
    - ＴＴＬ線
    - 電路板 X 3
    - 魔鬼氈 X 3
    - 熱縮套 X 1
- 技術：
    - 光敏電阻運用 ADC、compare、TIMER1 將亮度轉換成數位數值，並由相對變化 來判斷手勢
    - 呼吸燈透過平均步頻來設定 PWM 的 duty cycle，以改變呼吸頻率
    - 蜂鳴器運用 TIMER0 來調整警示聲響頻率
    - 超音波模組用 TIMER3 來計算超音波訊號來來回的時間換算出距離，並用 TIMER1 來計算平均步頻

## 周邊接口或Library及API使用說明
- 周邊接口：
    - pickit3 燒錄器接口
    - 行動電源接口
- Library：
    - xc.h
    - pic18f4520.h
    - stdio.h
- API：
    - 使用內建的__delay_us()、__delay_ms()，並且只在 main	function 裡使用

## 實際組員之分工項目
- 何鑑家
    - 打初審文件
    - 製作一排可變色 RGB LED 燈
    - 依據步頻改變呼吸燈呼吸頻率
    - 蜂鳴器
    - 協助使用超音波模組
    - 最後合併 code
    - 設計電路圖
    - 繪製系統架構圖
    - 繪製設計圖
    - 焊接電路
    - 撰寫系統文件
    - 購買材料
- 蔡明修
    - 題目發想
    - 校正初審文件
    - 利用超音波模組分析數據並計算出每分鐘步頻
    - 利用七段顯示器顯示步頻
    - 找出電路問題
    - 解決供電問題
    - 撰寫系統文件
    - 購買材料
    - 撰寫會議記錄
- 施彥安
    - 畫流程圖
    - 題目延伸設計
    - 提供田徑專業意見
    - 製作感光模組陣列判斷手勢
    - 超音波模組分析數據並計算每分鐘步頻
    - 合併七段顯示器、感光模組、超音波的 code
    - 檢測電路並找出電路問題
    - 解決供電問題
    - 購買材料
    - 撰寫系統文件
    - 剪接影片	
	
## 遇到的困難及如何解決
- Analog to	digital	腳位不夠
    - 困難：由於內建的內建的 ADC 轉換，一次只能針對一個腳位作轉換（一次只有 一個類比訊號輸入腳位），但我們的光敏電阻有三個。因此出現腳位不足情況。
    - 解決方式：每次在 interrupt 中取光敏電阻轉 digital 值時，將類比訊號輸入腳位 設為下一個腳位。（詳見 void__interrupt(high_priority)Hi_ISR(void)內 13 至 30 行）
- 光敏電阻判斷受環境限制：
    - 困難：一開始在判斷手滑的方向為左或右時，是將當下三個光敏電阻值與一定 值作比較。再依此結果判斷是否立起某些特定 flag，並進入下一個判斷。但由 於是定值，在某些光強度下會變得不靈敏。（光敏電阻值越暗越大）
    - 解決方式：光敏電阻亮暗的比較，由絕對變為相對。將原先的定值改為三個數 的平均數。由於被遮住的電阻值會與沒被遮住的電阻值相差極大（約 100~200），因此只有被遮住的光敏電阻會大於其平均數。使裝置再各種光照強 度下都能依都能一樣靈敏。
- Timer 數不夠：
    - 困難：由於我們的作品受到尺寸的限制，因此只能使用單一 Pic。但蜂鳴器、 adc、計算步頻、呼吸燈及超音波模組都需要 Timer，因此導致 Timer 不夠的問 題。
    - 解決方式：我們發現當 adc 在執行時，並不會用到計算步頻的功能。因此，我 們讓這兩個功能輪流使用 Timer1 來解決此問題。
- 供電問題：
    - 困難：由於我們的作品受到尺寸限制，因此電供應的體積不能太大。一開始計 畫用鈕扣電池作為供電。但經過一番嘗試後發現雖然電壓夠大，但電流太小。
    - 解決方式：大多數的電池都無法同時滿足體積小與電壓電流夠大的要求。因 此，我們找到一個超小的行動電源，電壓 5V，電流 1A	，同時滿足兩個要求。	

## 預期效益與結語
- 預期效益：有效幫助運動員意識到訓練全程每分每秒的即時步頻，藉以達到更精確的	訓練效果。
- 結語：能將上課學到的技術使用在日常生活中，並真正的解決一個需求，實在是個十分特別的經驗。
