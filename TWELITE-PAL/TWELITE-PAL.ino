/**
 * TWELITE PAL 環境センサーPAL の情報をM5Stack BASICに表示する。
 * TWELITE PAL、環境センサーPALは出荷状態のまま使用
 * UARTは Selial1(RX2-16,TX2-17)を使用
 */
#include <M5Stack.h>
#define DEBUG

// --- 表示用
#define TEXT_HEIGHT 30  // 1行の高さ(ピクセル数)

// --- TWELITEのデータ受信用
#define READ_DATA_LEN 50
uint8_t readData[READ_DATA_LEN];  // バイナリー形式の受信データ
int  recvLen = 0 ;  // 受信したデータのバイト数
bool recvFlag = false ;
bool recvMSB = true ;
static char bin2Hex[] = {"0123456789ABCDEF"} ;

struct TwelitePalEnv_s {  // TWELITE PAL　環境センサーからの情報保持用
  uint8_t devId ;       // デバイスID
  uint8_t serialNo[4];  // 固有ID
  uint8_t lqi ;         // 電波強度
  uint16_t volt ;       // 電源電圧
  int16_t temperature ;  // 気温
  uint16_t humidity ;    // 気圧
  int32_t brightness;   // 明るさ
} twelitePalEnv ;

// --- TWELITEコマンド送信用
#define CMD_DATA_LEN 14 // 送信コマンドの最大バイト数
#define CMD_STR_LEN 32  // 送信コマンドの最大文字数
uint8_t cmdData[CMD_DATA_LEN];
char cmdStr[CMD_STR_LEN] ;
static uint8_t cmd1[] = {7,0x78,0x88,0x01,0x01,0x38,0x71,0x00} ;
static uint8_t cmd2[] = {7,0x78,0x88,0x01,0x01,0x38,0x71,0x00} ;
static uint8_t cmd3[] = {7,0x78,0x88,0x01,0x01,0x38,0x71,0x00} ;

/**
 * １Byteのデータを16進数2文字のASCII文字列に変換。
 * ターミネーター(NULL)を付与する。
 */
void hexCharaSet(uint8_t data,char *buffPos) {
  buffPos[0] = bin2Hex[data >> 4] ;
  buffPos[1] = bin2Hex[data & 0x0F] ;
  buffPos[2] = 0x00 ;
}

/**
 * 16進数1桁のASCII文字を数値に変換。
 */
uint8_t hex2Bin(char c) {
  uint8_t hex ;
  if ('0' <= c && c <= '9') {
    hex = c - '0' ;
  } else
  if ('A' <= c && c <= 'F') {
    hex = c - 'A' + 10 ;
  } else
  if ('a' <= c && c <= 'f') {
    hex = c - 'a' + 10 ;
  }
  return hex ;
}

/**
 * TWELITEに送信するコマンド生成して、cmdStrに格納する。
 * cmdNo : 1-未定 2-未定 3-未定
 */
void sendCommandBuild(int cmdNo) {
  memset(cmdData,0,CMD_DATA_LEN) ;
  int cmdLen ;
  uint8_t *sedData ;
  switch(cmdNo) {
    case 1: cmdLen = cmd1[0] ; sedData = &cmd1[1] ; break ;
    case 2: cmdLen = cmd2[0] ; sedData = &cmd2[1] ; break ;
    case 3: cmdLen = cmd3[0] ; sedData = &cmd3[1] ; break ;
  }
  uint8_t checkSum = 0 ;
  cmdStr[0] = ':' ;
  for (int i=0;i<cmdLen;i++) {
    checkSum += sedData[i] ;
    hexCharaSet(sedData[i],&cmdStr[1+i*2]) ;
  }
  checkSum = ~checkSum + 1 ;
  hexCharaSet(checkSum,&cmdStr[1+cmdLen*2]) ;
  cmdStr[1+(cmdLen+1)*2] = 0x0d ;
  cmdStr[1+(cmdLen+1)*2+1] = 0x0a ;
  cmdStr[1+(cmdLen+1)*2+2] = 0x00 ;
}

/**
 * TWELITE PAL 環境センサーからの受信データをデコードして
 * twelitePalEnv に格納する。
 * return : False - チェックサムエラー
 */
bool readDataDecode(int dataLen , uint8_t *sensorData) {

#ifdef DEBUG
  char dispStr[4] ;
  Serial.print(":") ;
  for (int i=0;i<dataLen;i++) {
    sprintf(dispStr,"%02X",sensorData[i]) ;
    Serial.print(dispStr) ;
  }
  Serial.println() ;
#endif

  // -- CheckSum
  uint8_t ch1 = 0 ; // チェックサム１：CRC8
  uint8_t ch2 = 0 ; // チェックサム２：LRC
  uint8_t MSB_CRC8 = 0x31; // X^8+X^5+X^4+1 : CRC8-Maxim
  for (int i=0;i<dataLen-2;i++) {
    ch1 ^= sensorData[i] ;
    for ( int b = 0 ; b < CHAR_BIT ; b++ ){
      if ( ch1 & 0x80 ){
        ch1 = (ch1 << 1) ^ MSB_CRC8;
      } else {
        ch1 <<= 1;
      }
    }
  }
  for (int i=0;i<dataLen-1;i++) {
    ch2 += sensorData[i] ;
  }
  ch2 = ~ch2 + 1 ;
  if (ch1 != sensorData[dataLen-2] || ch2 != sensorData[dataLen-1]) {
#ifdef DEBUG
    Serial.print("CHECK SUM ERROR") ;
#endif
    return false;
  }

  memset((char *)&twelitePalEnv,0,sizeof(struct TwelitePalEnv_s)) ;
  
  // 送信デバイスID
  twelitePalEnv.devId = sensorData[11] ;
  // 送信元個体識別番号
  for (int i=0;i<4;i++) {
    twelitePalEnv.serialNo[i] = sensorData[i+7] ;
  }
  // 電波強度
  twelitePalEnv.lqi = sensorData[4] ;

  // --- チャンク
  int cDnum = sensorData[14] ;
  int dp = 15 ;
  for (int cn = 0 ; cn < cDnum ; cn ++) {
    int16_t data16 = 0 ;
    uint16_t dataU16 = 0;
    int32_t data32 = 0 ;

    uint8_t dataType = sensorData[dp] ; dp++ ;
    uint8_t dataSource = sensorData[dp] ; dp++ ;
    uint8_t exData = sensorData[dp] ; dp++ ;
    uint8_t dataLen = sensorData[dp] ; dp++ ;

#ifdef DEBUG
    Serial.print("cn=") ;
    Serial.print(cn) ;
    Serial.print("  DataBit=") ;
    Serial.println(readData[dp],HEX) ;
    Serial.print("dataType=") ;
    Serial.print(dataType,HEX) ;
    Serial.print("  dataLen=") ;
    Serial.print(dataLen,HEX) ;
    Serial.print("  dataSource=") ;
    Serial.print(dataSource,HEX) ;
    Serial.print("  exData=") ;
    Serial.println(exData,HEX) ;
#endif

    if (dataLen == 2) {
//      data16 = sensorData[dp] << 8 | sensorData[dp+1] ;
//      dp += 2 ;
      ((int8_t *)&data16)[1] = sensorData[dp] ; ((uint8_t *)&dataU16)[1] = sensorData[dp] ; dp++ ;
      ((int8_t *)&data16)[0] = sensorData[dp] ; ((uint8_t *)&dataU16)[0] = sensorData[dp] ; dp++ ;
    } else
    if (dataLen == 4) {
//      data32 = sensorData[dp] << 24 | sensorData[dp+1] << 16 | sensorData[dp+2] << 8 | sensorData[dp+3] ;
//      dp += 4 ;
      ((uint8_t *)&data32)[3] = sensorData[dp] ; dp++ ;
      ((uint8_t *)&data32)[2] = sensorData[dp] ; dp++ ;
      ((uint8_t *)&data32)[1] = sensorData[dp] ; dp++ ;
      ((uint8_t *)&data32)[0] = sensorData[dp] ; dp++ ;
    } else {
      dp += dataLen ;
    }
#ifdef DEBUG
    Serial.print("data16=") ;
    Serial.print(data16) ;
    Serial.print("  dataU16=") ;
    Serial.print(dataU16) ;
    Serial.print("  data32=") ;
    Serial.println(data32) ;
#endif

    if (dataSource == 0x30 && exData == 0x08) {
      // 電源電圧
      twelitePalEnv.volt = dataU16 ;
    } else
    if (dataSource == 0x01 && exData == 0x00) {
      // 温度
      twelitePalEnv.temperature = data16 ;
    } else
    if (dataSource == 0x02 && exData == 0x00) {
      // 湿度
      twelitePalEnv.humidity = dataU16 ;
    } else
    if (dataSource == 0x03 && exData == 0x00) {
      // 明るさ
      twelitePalEnv.brightness = data32 ;
    }
  }
#ifdef DEBUG
  Serial.print("  temperature=") ;
  Serial.print(twelitePalEnv.temperature) ;
  Serial.print("  humidity=") ;
  Serial.print(twelitePalEnv.humidity) ;
  Serial.print("  brightness=") ;
  Serial.println(twelitePalEnv.brightness) ;
#endif

  return true ;
}

/**
 * TWELITE PAL 環境センサーの情報表示
 */
void statDisp() {
  int xP = 0;
  int yP = TEXT_HEIGHT+1;
  char dispStr[24] ;

//  M5.Lcd.fillRect(0, yP , 320, TEXT_HEIGHT*5 , TFT_BLACK);  // なくても平気なので、チラツキ防止のために外す
  M5.Lcd.setTextColor(TFT_GREENYELLOW, TFT_BLACK);

  // -----
  // 送信デバイスID
  yP = TEXT_HEIGHT + 1 ;
  xP = M5.Lcd.drawString("Device:", 0 , yP , 4);
  hexCharaSet(twelitePalEnv.devId,dispStr) ;
  xP += M5.Lcd.drawString(dispStr, xP, yP, 4);
  xP += M5.Lcd.drawString("-", xP , yP , 4);
  // 送信元個体識別番号
  for (int i=0;i<4;i++) {
    hexCharaSet(twelitePalEnv.serialNo[i],dispStr) ;
    xP += M5.Lcd.drawString(dispStr, xP, yP, 4);
  }

  // 電波強度
  yP += TEXT_HEIGHT ;
  xP = M5.Lcd.drawString("LQI:", 0 , yP , 4);
  sprintf(dispStr,"%03d",twelitePalEnv.lqi) ;
  xP += M5.Lcd.drawString(dispStr, xP, yP, 4);

  // 電圧
  xP += M5.Lcd.drawString(" / Vcc:", xP , yP , 4);
  float vcc = twelitePalEnv.volt / 1000. ;
  sprintf(dispStr,"%1.2f ",vcc) ;
  xP += M5.Lcd.drawString(dispStr, xP, yP, 4);
  // -----

  // 明るさ
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  xP = 0 ;
  yP += TEXT_HEIGHT ;
  xP += M5.Lcd.drawString("brightness:", xP , yP , 4);
  sprintf(dispStr,"%d     ",twelitePalEnv.brightness) ;
  xP += M5.Lcd.drawString(dispStr, xP, yP, 4);

  // --- 温湿度算出
  float humidityDisp = twelitePalEnv.humidity / 100. ;
  float temperatureDisp = twelitePalEnv.temperature /100.;

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  xP = 0 ;
  yP += TEXT_HEIGHT ;
  sprintf(dispStr,"%2.1f  ",temperatureDisp) ;
  M5.Lcd.setTextFont(7);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(12,yP);
  M5.Lcd.print(dispStr);

  sprintf(dispStr,"%2.1f  ",humidityDisp) ;
  M5.Lcd.setCursor(166,yP);
  M5.Lcd.print(dispStr);

  M5.Lcd.setTextFont(4);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(130,yP+26);

  M5.Lcd.print("C");
  M5.Lcd.drawCircle(127, yP+26 , 4, WHITE);
  M5.Lcd.drawCircle(127, yP+26 , 3, WHITE);

  M5.Lcd.setCursor(280,yP+26);
  M5.Lcd.print("%");
}

void setup() {
  M5.begin();
  M5.Power.begin();  // Init Power module.
  M5.Speaker.setVolume(3);
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextFont(4);
  M5.Lcd.setTextSize(1);

#ifdef DEBUG
  Serial.begin(115200) ;
#endif
  Serial1.begin(115200, SERIAL_8N1, 16, 17);  // シリアル通信1初期化(RX, TX)

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.fillRect(0, 0, 320, TEXT_HEIGHT, TFT_BLUE);
  M5.Lcd.drawCentreString("TWELITE PAL(ENV)", 320 / 2, 4, 4);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
}

void loop() {
  M5.update();  //本体のボタン状態更新
  // データ送信
  if (M5.BtnA.wasPressed()) { // コマンド送信
    sendCommandBuild(1);
    Serial1.print(cmdStr) ;
  }

  // データ受信
  if (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == ':') {
      memset(readData,0,READ_DATA_LEN) ;
      recvFlag = true ;
      recvLen = 0 ;
    } else
    if (c == 0x0d) {
      // STATUS RECIVE
      recvFlag = false ;

#ifdef DEBUG
Serial.print("recvLen=") ;
Serial.print(recvLen) ;
Serial.print("  readData[12]=") ;
Serial.print(readData[12],HEX) ;
Serial.print("  readData[13]=") ;
Serial.println(readData[13],HEX) ;
#endif

      if (recvLen == 49 && readData[12] == 0x80 && readData[13] == 0x82) {
        // 環境センサーPAL
        if (readDataDecode(recvLen , readData)) {
          statDisp() ;
        }
      }
    } else {
      if (c >= ' ') {
        if (recvMSB) {
          readData[recvLen] = hex2Bin(c) << 4 ;
          recvMSB = false ;
        } else {
          readData[recvLen] |= hex2Bin(c) ;
          recvMSB = true ;
          recvLen++ ;
        }
      }
    }
  }
}
