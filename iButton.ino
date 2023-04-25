/*
  Скетч к проекту "Копировальщик ключей для домофона RFID с OLED дисплеем и хранением 8 ключей в память EEPROM"
  Аппаратная часть построена на Arduino Nano
  Исходники на GitHub: https://github.com/AlexMalov/EasyKeyDublicatorRFID_OLED/
  Автор: МЕХАТРОН DIY, AlexMalov, 2019
  v3.2 - fix cyfral bug
  v3.3 - OLED -> LCD1602, GyverEncoder -> EncButton, вывод на экран ID ключа и режима работы
*/

#include <OneWire.h>
//#include <OneWireSlave.h>
#include "pitches.h"
#include <EEPROM.h>
#define _LCD_TYPE 1                      // выставляем тип LCD - с шиной I2C
#include <LCD_1602_RUS_ALL.h>            // включаем библиотеку LCD с поддержкой кириллицы 
LCD_1602_RUS lcd(0x27, 16, 2);           // Адрес LCD (0x27) и параметры экрана (16х2)

//extern uint8_t SmallFont[];
//extern uint8_t BigNumbers[];
#include "EncButton.h"
#include "TimerOne.h"

//массив универсальных ключей из темы - https://4pda.to/forum/index.php?showtopic=953401&st=340#entry120324032
byte UniKey[22][8] = 
{ 
{0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0xFF, 0x00}, // - Univer 1F
{0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x2F, 0xFF, 0x00}, // - Univer 2F
{0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x9B}, // - UK-1 Metakom 2003
{0x01, 0xBE, 0x40, 0x11, 0x5A, 0x36, 0x00, 0xE1}, // - UK-2 Vizit – код универсального ключа, для Vizit
{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3D}, // - UK-3 Cyfral
{0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x2F}, // - Стандартный универсальный ключ
{0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, // - Обычный
{0x01, 0x00, 0x00, 0x00, 0x00, 0x90, 0x19, 0xFF}, // - Отлично работает на старых домофонах
{0x01, 0x53, 0xD4, 0xFE, 0x00, 0x00, 0x7E, 0x88}, // - Cyfral, Metakom
{0x01, 0x53, 0xD4, 0xFE, 0x00, 0x00, 0x7E, 0x00}, // - Cyfral,Metakom
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x14}, // - Открываает 98% Metakom и некоторые Cyfral
{0x01, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00}, // - домофоны Cyfral + фильтр и защита
{0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00}, // - Metakom
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xA0}, // - Metakom 95%
{0x01, 0x00, 0xBE, 0x11, 0xAA, 0x00, 0x00, 0xFB}, // - домофоны KeyMan
{0x01, 0xBE, 0x40, 0x11, 0x0A, 0x00, 0x00, 0x1D}, // - проверен работает Vizit иногда KeyMan
{0x01, 0x53, 0xD4, 0xFE, 0x00, 0x00, 0x00, 0x6F}, // - домофоны Vizit - до 99%
{0x01, 0xBE, 0x40, 0x11, 0x5A, 0x36, 0x00, 0x00}, // - Vizit 99%
{0x01, 0x76, 0xB8, 0x2E, 0x0F, 0x00, 0x00, 0x5C}, // - домофоны Форвард
{0x01, 0xA9, 0xE4, 0x3C, 0x09, 0x00, 0x00, 0x00}, // - домофоны Eltis - до 90%
{0x01, 0xBE, 0x40, 0x11, 0x5A, 0x56, 0x00, 0xBB}, // - проверен работает
{0x01, 0xBE, 0x40, 0x11, 0x00, 0x00, 0x00, 0x77}, // - проверен работает
//{0x01, 0x2C, 0x7A, 0x0F, 0xA0, 0x00, 0x00, 0x4A}, // - ????
//{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // - домофоны Cyfral 70%
//{0x01, 0x6F, 0x2E, 0x88, 0x8A, 0x00, 0x00, 0x4D}, // - Открывать что-то должен
//{0x01, 0x71, 0xA8, 0x75, 0x0F, 0x00, 0x00, 0xE9}, // - Vizit 55%
//{0x11, 0xBE, 0x40, 0x11, 0x5B, 0x00, 0x00, 0xCD}, // - Vizit 50%
//{0x0F, 0xBE, 0x40, 0x11, 0x5A, 0x36, 0x00, 0x9E}, // - Vizit 50%
};
//идентификаторы унив ключей для отображения
String NameKey[22] = {"Uni_1F", "Uni_2F", "UK1 Met", "UK2 Viz", "UK3 Cyf", "Std", "Std2", "Std_Old", "CyfMet1", "CyfMet2", "CyfMet3","Cyf_Flt", "Met_1", "Met_2", "KeyMan", "KeyMViz", "Vizit1", "Vizit2", "Forward", "Eltis", "Wrk_1", "Wrk_2"};

//settings
#define rfidUsePWD 0        // ключ использует пароль для изменения
#define rfidPWD 123456      // пароль для ключа
#define rfidBitRate 2       // Скорость обмена с rfid в kbps

//pins
#define iButtonPin A3      // Линия data ibutton
#define iBtnEmulPin A1     // Линия эмулятора ibutton
#define Luse_Led 13        // Светодиод лузы
#define R_Led 2            // RGB Led
#define G_Led 3
#define B_Led 4
#define ACpin 6            // Вход Ain0 аналогового компаратора 0.1В для EM-Marie 
#define speakerPin 12       // Спикер, он же buzzer, он же beeper
#define FreqGen 11         // генератор 125 кГц
#define CLK 8              // s1 энкодера
#define DT 9               // s2 энкодера
#define BtnPin 10          // Кнопка переключения режима чтение/запись

EncButton<EB_TICK, CLK, DT, BtnPin> enc1; // инициализируем энкодер
OneWire ibutton (iButtonPin);
//OneWireSlave iBtnEmul(iBtnEmulPin);       //Эмулятор iButton для BlueMode
byte maxKeyCount;                         // максимальное кол-во ключей, которое влазит в EEPROM, но не > 20
byte EEPROM_key_count;                    // количество ключей 0..maxKeyCount, хранящихся в EEPROM
byte EEPROM_key_index = 0;                // 1..EEPROM_key_count номер текущего ключа в EEPROM  
byte addr[8];                             // временный буфер
byte keyID[8];                            // ID ключа для записи
byte rfidData[5];                         // значащие данные frid em-marine
byte halfT;                               // полупериод для метаком
enum emRWType {rwUnknown, TM01, RW1990_1, RW1990_2, TM2004, T5557, EM4305};               // тип болванки
enum emkeyType {keyUnknown, keyDallas, keyTM2004, keyCyfral, keyMetacom, keyEM_Marine, Embedded};   // тип оригинального ключа  
emkeyType keyType;
enum emMode {md_empty, md_read, md_write, md_blueMode, md_write_uni};                                   // режим работы копировальщика
emMode copierMode = md_empty;

String lcd_print_mode = "";                  // строка для записи отображения режима работы

byte UNI_key_count;                        // количество универсальных ключей
int UNI_key_index = 0;                     // 1..UNI_key_count - номер текущего универсального ключа
byte keyID_temp[8];                        // временный буфер для записи ID ключа из режима "W" когда находимся в режиме "U"

byte rightH_flag = 0;                      // флаг для ограничения кол-ва событий enc1.rightH и переключения между экранами
byte scroll_flag = 0;                      // флаг для ограничения кол-ва срабатываний скрола при событиях enc1.left и enc1.right

void scrollMessage(byte row, String message, int delayTime) {  // функция вывода бегущей строки
  for (byte i=0; i < 16; i++) {                                // 16 - количество столбцов в экране
    message = " " + message;  
  } 
  message = message + " "; 
  for (byte position = 0; position < message.length(); position++) {
    lcd.setCursor(0, row);
    lcd.print(message.substring(position, position + 16));
    delay(delayTime);
  }
}

void LCD_printKey(byte buf[8], byte msgType = 0){
  String st;
  switch (msgType){
    case 0: lcd.clear(); lcd.setCursor(0,0); st = "пам: " + String(EEPROM_key_index) + " из " + String(EEPROM_key_count); lcd.print(st); lcd.setCursor(14,0); lcd.print(lcd_print_mode); break;      
    case 1: lcd.clear(); lcd.setCursor(0,0); st = "Hold Btn to save"; lcd.print(st); break; 
    case 3: lcd.clear(); lcd.setCursor(0,0); st = "В памяти -> " + String(indxKeyInROM(buf)); lcd.print(st); break;
    case 4: lcd.clear(); lcd.setCursor(0,0); st = "пам: " + String(UNI_key_index+1) + " из " + String(UNI_key_count+1); lcd.print(st); lcd.setCursor(14,0); lcd.print(lcd_print_mode); keyType = Embedded; break; 
  }
  st = "тип: ";
  switch (keyType){
    case keyDallas: st += "Dallas"; break;      
    case keyCyfral: st += "Cyfral";  break;  
    case keyMetacom: st += "Metakom"; break;             
    case keyEM_Marine: st += "EM_Marine"; break;
    case keyUnknown: st += "Unknown"; break;
    case Embedded: st = "имя: " + NameKey[UNI_key_index]; break;
  }
  lcd.setCursor(0,1);
  lcd.print(st); // распечатаем тип ключа
}

void LCD_printError(String st, bool err = true){
  lcd.clear();
  if (err) { lcd.setCursor(0,6); lcd.print(F("Ошибка!")); }
    else { lcd.setCursor(0,7); lcd.print(F("OK")); }
  lcd.setCursor(0,1);
  lcd.print(st);
}

void setup() {
  pinMode(Luse_Led, OUTPUT);
//  myOLED.begin(SSD1306_128X32); //инициализируем дисплей
  lcd.init();                                       // Инициализируем дисплей 16x2
  lcd.backlight();                                  // Включаем подсветку
  pinMode(BtnPin, INPUT_PULLUP);                            // включаем чтение и подягиваем пин кнопки режима к +5В
  pinMode(speakerPin, OUTPUT);
  pinMode(ACpin, INPUT);                                    // Вход аналогового компаратора 3В для Cyfral 
  pinMode(R_Led, OUTPUT); pinMode(G_Led, OUTPUT); pinMode(B_Led, OUTPUT);  //RGB-led
  //clearLed();
  pinMode(FreqGen, OUTPUT);                               
  Serial.begin(9600);
// LDC1602 - start
  lcd.clear();
  lcd.setCursor(0,0);                                         // Устанавливаем курсор в 1й символ 1 строки
  lcd.print(F("Приложите ключ.."));
// LDC1602 - end
//  myOLED.clrScr();                                          // Очищаем буфер дисплея.
//  myOLED.setFont(SmallFont);                                // Перед выводом текста необходимо выбрать шрифт
//  myOLED.print(F("Hello, read a key..."), LEFT, 0);
//  char st[16] = {98, 121, 32, 77, 69, 88, 65, 84, 80, 79, 72, 32, 68, 73, 89, 0};
//  myOLED.print(st, LEFT, 24);
//  myOLED.update();
  Sd_StartOK();                                               // звук "Успешное включение"

  EEPROM_key_count = EEPROM[0];                               // считаем количество EEPROM ячеек
  UNI_key_count = sizeof(UniKey)/sizeof(UniKey[0])-1;         // считаем количество ячеек универсальных кодов

  maxKeyCount = EEPROM.length() / 8 - 1; if (maxKeyCount > 20) maxKeyCount = 20;
  if (EEPROM_key_count > maxKeyCount) EEPROM_key_count = 0;
  if (EEPROM_key_count != 0 ) {
    EEPROM_key_index = EEPROM[1];
    Serial.println("Reading keys from EEPROM:");
    Serial.println(F("-----------------------"));
    for (byte j = 1; j <= EEPROM_key_count; j++) {
      EEPROM_get_key(j, keyID);
      for (byte i = 0; i < 8; i++) {
        Serial.print(keyID[i], HEX); if (i<7) Serial.print(":");  
      }
    Serial.println();
    }
    Serial.println(F("-----------------------"));
    copierMode = md_blueMode;                                 // задаем начальное значение режима работы. при первом щелчке энкодера сменится на 1й режим - U
    LCD_printKey(keyID);
    clearLed();
  } else {
    Serial.print(F("ROM has no keys yet."));
    lcd.print(F("ROM has no keys yet."));
  }

  Timer1.initialize(1000);                                   // установка таймера на каждые 1000 микросекунд (= 1 мс)
  Timer1.attachInterrupt(timerIsr);                          // запуск таймера
}

void timerIsr() {                                            // прерывание таймера для энкодера
  enc1.tick();     
}

void clearLed(){
  digitalWrite(R_Led, LOW);
  digitalWrite(G_Led, LOW);
  digitalWrite(B_Led, LOW);
  switch (copierMode)
  {
    case md_read: digitalWrite(G_Led, HIGH); digitalWrite(Luse_Led, HIGH); break;
    case md_write: digitalWrite(R_Led, HIGH); digitalWrite(Luse_Led, HIGH); break; 
    case md_blueMode: digitalWrite(B_Led, HIGH); digitalWrite(Luse_Led, LOW);break; // read key
    case md_write_uni: digitalWrite(R_Led, HIGH); digitalWrite(Luse_Led, HIGH); break;
  }
}

byte indxKeyInROM(byte buf[]){                              // возвращает индекс или ноль если нет в ROM
  byte buf1[8]; bool eq = true;
  for (byte j = 1; j<=EEPROM_key_count; j++){               // ищем ключ в eeprom. 
    EEPROM.get(j*sizeof(buf1), buf1);
    for (byte i = 0; i < 8; i++) 
      if (buf1[i] != buf[i]) { eq = false; break;}
    if (eq) return j;
    eq = true;
  }
  return 0;
}

bool EPPROM_AddKey(byte buf[]){                            // функция запоминания ключа
  byte buf1[8]; byte indx;
  indx = indxKeyInROM(buf);                                // ищем ключ в eeprom. Если находим, то не делаем запись, а индекс переводим в него
  if ( indx != 0) { 
    EEPROM_key_index = indx;
    EEPROM.update(1, EEPROM_key_index);
    return false; 
  }
  if (EEPROM_key_count <= maxKeyCount) EEPROM_key_count++;
  if (EEPROM_key_count < maxKeyCount) EEPROM_key_index = EEPROM_key_count;
    else EEPROM_key_index++;
  if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
  Serial.println(F("Adding to EEPROM"));
  for (byte i = 0; i < 8; i++) {
    buf1[i] = buf[i];
    Serial.print(buf[i], HEX); if (i<7) Serial.print(F(":"));  
  }
  Serial.println();
  EEPROM.put(EEPROM_key_index*sizeof(buf1), buf1);
  EEPROM.update(0, EEPROM_key_count);
  EEPROM.update(1, EEPROM_key_index);
  return true;
}

void EEPROM_get_key(byte EEPROM_key_index1, byte buf[8]){
  byte buf1[8];
  int address = EEPROM_key_index1*sizeof(buf1);
  if (address > EEPROM.length()) return;
  EEPROM.get(address, buf1);
  for (byte i = 0; i < 8; i++) buf[i] = buf1[i];
  keyType = getKeyType(buf1);
}

emkeyType getKeyType(byte* buf){
  if (buf[0] == 0x01) return keyDallas;                       // это ключ формата dallas
  if ((buf[0] >> 4) == 0b0001) return keyCyfral;
  if ((buf[0] >> 4) == 0b0010) return keyMetacom;
  if ((buf[0] == 0xFF) && vertEvenCheck(buf)) return keyEM_Marine;
  return keyUnknown;
}

//*************** dallas **************
emRWType getRWtype(){    
   byte answer;
  // TM01 это неизвестный тип болванки, делается попытка записи TM-01 без финализации для dallas или c финализацией под cyfral или metacom
  // RW1990_1 - dallas-совместимые RW-1990, RW-1990.1, ТМ-08, ТМ-08v2 
  // RW1990_2 - dallas-совместимая RW-1990.2
  // TM2004 - dallas-совместимая TM2004 в доп. памятью 1кб
  // пробуем определить RW-1990.1
  ibutton.reset(); ibutton.write(0xD1);                      // пробуем снять флаг записи для RW-1990.1
  ibutton.write_bit(1);                                      // записываем значение флага записи = 1 - отключаем запись
  delay(10); pinMode(iButtonPin, INPUT);
  ibutton.reset(); ibutton.write(0xB5);                      // send 0xB5 - запрос на чтение флага записи
  answer = ibutton.read();
  //Serial.print(F("\n Answer RW-1990.1: ")); Serial.println(answer, HEX);
  if (answer == 0xFE){
    Serial.println(F("Type:             Dallas RW-1990.1 "));
    return RW1990_1;            // это RW-1990.1
  }
  // пробуем определить RW-1990.2
  ibutton.reset(); ibutton.write(0x1D);                      // пробуем установить флаг записи для RW-1990.2 
  ibutton.write_bit(1);                                      // записываем значение флага записи = 1 - включаем запись
  delay(10); pinMode(iButtonPin, INPUT);
  ibutton.reset(); ibutton.write(0x1E);                      // send 0x1E - запрос на чтение флага записи
  answer = ibutton.read();
  if (answer == 0xFE){
    ibutton.reset(); ibutton.write(0x1D);                    // возвращаем оратно запрет записи для RW-1990.2
    ibutton.write_bit(0);                                    // записываем значение флага записи = 0 - выключаем запись
    delay(10); pinMode(iButtonPin, INPUT);
    Serial.println(F("Type:             Dallas RW-1990.2 "));
    return RW1990_2; // это RW-1990.2
  }
  // пробуем определить TM-2004
  ibutton.reset(); ibutton.write(0x33);                      // посылаем команду чтения ROM для перевода в расширенный 3-х байтовый режим
  for ( byte i=0; i<8; i++) ibutton.read();                  // читаем данные ключа
  ibutton.write(0xAA);                                       // пробуем прочитать регистр статуса для TM-2004    
  ibutton.write(0x00); ibutton.write(0x00);                  // передаем адрес для считывания
  answer = ibutton.read();                                   // читаем CRC комманды и адреса
  byte m1[3] = {0xAA, 0,0};                                  // вычисляем CRC комманды
  if (OneWire::crc8(m1, 3) == answer) {
    answer = ibutton.read();                                 // читаем регистр статуса
    //Serial.print(" status: "); Serial.println(answer, HEX);
    Serial.println(F("Type:             Dallas TM2004"));
    ibutton.reset();
    return TM2004; // это Type: TM2004
  }
  ibutton.reset();
  Serial.println(F("Type:             Dallas unknown, trying TM-01! "));
  return TM01;                              // это неизвестный тип DS1990, нужно перебирать алгоритмы записи (TM-01)
}

bool write2iBtnTM2004(){                // функция записи на TM2004
  byte answer; bool result = true;
  ibutton.reset();
  ibutton.write(0x3C);                                       // команда записи ROM для TM-2004    
  ibutton.write(0x00); ibutton.write(0x00);                  // передаем адрес с которого начинается запись
  for (byte i = 0; i<8; i++){
    digitalWrite(R_Led, !digitalRead(R_Led));
    ibutton.write(keyID[i]);
    answer = ibutton.read();
    //if (OneWire::crc8(m1, 3) != answer){result = false; break;}     // crc не верный
    delayMicroseconds(600); ibutton.write_bit(1); delay(50);         // испульс записи
    pinMode(iButtonPin, INPUT);
    Serial.print('*');
    Sd_WriteStep();
    if (keyID[i] != ibutton.read()) { result = false; break;} //читаем записанный байт и сравниваем, с тем что должно записаться
  } 
  if (!result){
    ibutton.reset();
    Serial.println("copy failed");
    Serial.println(F("-----------------------"));
    LCD_printError(F("ошибка копир."));
    Sd_ErrorBeep();
    digitalWrite(R_Led, HIGH);
    return false;    
  }
  ibutton.reset();
  Serial.println(F(" was copied successfully"));
  Serial.println(F("-----------------------"));
  LCD_printError(F("ключ скопирован"), false);
  Sd_ReadOK();
  delay(2000);
  digitalWrite(R_Led, HIGH);
  return true;
}

bool write2iBtnRW1990_1_2_TM01(emRWType rwType){              // функция записи на RW1990.1, RW1990.2, TM-01C(F)
  byte rwCmd, bitCnt = 64, rwFlag = 1;
  switch (rwType){
    case TM01: rwCmd = 0xC1; if ((keyType == keyMetacom)||(keyType == keyCyfral)) bitCnt = 36; break;                   //TM-01C(F)
    case RW1990_1: rwCmd = 0xD1; rwFlag = 0; break;           // RW1990.1  флаг записи инвертирован
    case RW1990_2: rwCmd = 0x1D; break;                       // RW1990.2
  }
  ibutton.reset(); ibutton.write(rwCmd);                      // send 0xD1 - флаг записи
  ibutton.write_bit(rwFlag);                                  // записываем значение флага записи = 1 - разрешить запись
  delay(5); pinMode(iButtonPin, INPUT);
  ibutton.reset(); 
  if (rwType == TM01) ibutton.write(0xC5);
    else ibutton.write(0xD5);                                 // команда на запись
  if (bitCnt == 36) BurnByteMC(keyID);
  else for (byte i = 0; i< (bitCnt >> 3); i++){
    digitalWrite(R_Led, !digitalRead(R_Led));
    if (rwType == RW1990_1) BurnByte(~keyID[i]);              // запись происходит инверсно для RW1990.1
      else BurnByte(keyID[i]);
    Serial.print('*');
    Sd_WriteStep();
  }

  if (bitCnt == 64) {
      ibutton.write(rwCmd);                                   // send 0xD1 - флаг записи
      ibutton.write_bit(!rwFlag);                             // записываем значение флага записи = 1 - отключаем запись
      delay(5); pinMode(iButtonPin, INPUT);
    }
  digitalWrite(R_Led, LOW);       
  if (!dataIsBurningOK(bitCnt)){                              // проверяем корректность записи
    Serial.println("\ncopy failed");
    Serial.println(F("-----------------------"));
    LCD_printError(F("ошибка копир."));
    Sd_ErrorBeep();
    digitalWrite(R_Led, HIGH);
    return false;
  }
  Serial.println(F(" was copied successfully"));
  Serial.println(F("-----------------------"));
  if ((keyType == keyMetacom)||(keyType == keyCyfral)){       // переводим ключ из формата dallas
    ibutton.reset();
    if (keyType == keyCyfral) ibutton.write(0xCA);            // send 0xCA - флаг финализации Cyfral
      else ibutton.write(0xCB);                               // send 0xCB - флаг финализации metacom
    ibutton.write_bit(1);                                     // записываем значение флага финализации = 1 - перевезти формат
    delay(10); pinMode(iButtonPin, INPUT);
  }
  LCD_printError(F("ключ скопирован"), false);
  Sd_ReadOK();
  delay(2000);
  digitalWrite(R_Led, HIGH);
  return true;
}

void BurnByte(byte data){
  for(byte n_bit = 0; n_bit < 8; n_bit++){ 
    ibutton.write_bit(data & 1);  
    delay(5);                        // даем время на прошивку каждого бита до 10 мс
    data = data >> 1;                // переходим к следующему bit
  }
  pinMode(iButtonPin, INPUT);
}

void BurnByteMC(byte buf[8]){
  byte j = 7;
  for(byte n_bit = 0; n_bit < 36; n_bit++){ 
    ibutton.write_bit(((~buf[n_bit>>3]) >> j ) & 1);  
    delay(5);                        // даем время на прошивку каждого бита 5 мс
    if (j==0) j = 8; j--;
  }
  pinMode(iButtonPin, INPUT);
}

void convetr2MC(byte buff[8]){
  byte data;
  for (byte i = 0; i < 5; i++){
    data = ~buff[i];
    buff[i] = 0;
    for (byte j = 0; j < 8; j++) 
      if ( (data>>j)&1) bitSet(buff[i], 7-j);
  }
  buff[4] &= 0xf0;  buff[5] = 0; buff[6] = 0; buff[7] = 0;
}

bool dataIsBurningOK(byte bitCnt){
  byte buff[8];
  if (!ibutton.reset()) return false;
  ibutton.write(0x33);
  ibutton.read_bytes(buff, 8);
  if (bitCnt == 36) convetr2MC(buff);
  byte Check = 0;
  Serial.println();
  for (byte i = 0; i < 8; i++){ 
    if (keyID[i] == buff[i]) Check++;       // сравниваем код для записи с тем, что уже записано в ключе.
    Serial.print(buff[i], HEX); if (i<7) Serial.print(":");
  }
  if (Check != 8) return false;             // если коды совпадают, ключ успешно скопирован
  return true;
}

bool write2iBtn(){
  int Check = 0;
  if (!ibutton.search(addr)) { 
    ibutton.reset_search(); 
    return false;
  }
  Serial.println(F("-----------------------"));
  Serial.println(F("Getting ready to burn"));
  Serial.println(F("-----------------------"));
  Serial.print(F("Existing code is: "));
  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); if (i < 7 ) Serial.print(":");  
    if (keyID[i] == addr[i]) Check++;         // сравниваем код для записи с тем, что уже записано в ключе.
  }
  Serial.println();
  if (Check == 8) {                           // если коды совпадают, ничего писать не нужно
    digitalWrite(R_Led, LOW);
    Serial.println(F("It is the same key. Writing is not needed."));
    Serial.println(F("-----------------------")); 
    LCD_printError(F("ключ идентичен"));
    Sd_ErrorBeep();
    digitalWrite(R_Led, HIGH);
    delay(1000);
    return false;
  }
  emRWType rwType = getRWtype();                    // определяем тип RW-1990.1 или 1990.2 или TM-01
  Serial.println(F("-----------------------"));
  Serial.print(F("Burning iButton: "));
  if (rwType == TM2004) return write2iBtnTM2004();  //шьем TM2004
    else return write2iBtnRW1990_1_2_TM01(rwType);  //пробуем прошить другие форматы
  Serial.println();
}

bool searchIbutton(){
  if (!ibutton.search(addr)) {                        // проверяем, если в кнопке уже есть прочитанный ключ
    ibutton.reset_search(); 
    return false;
  }
  Serial.println(F("-----------------------"));
  Serial.print(F("Current key:      "));
  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); if (i<7) Serial.print(":");
    keyID[i] = addr[i];                               // копируем прочтенный код в ReadID
  }
  Serial.println();
  if (addr[0] == 0x01) {                              // это ключ формата dallas
    keyType = keyDallas;
    if (getRWtype() == TM2004) keyType = keyTM2004;
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println(F("!! CRC is not valid !!"));
      Serial.println(F("-----------------------"));
      LCD_printError(F("CRC не верно!"));
      Sd_ErrorBeep();
      digitalWrite(B_Led, HIGH);
      return false;
    }
    return true;
  }
  switch (addr[0]>>4){
    case 1: Serial.println(F("Type:             maybe Cyfral in Dallas key")); break;      
    case 2: Serial.println(F("Type:             maybe Metacom in Dallas key"));  break;  
    case 3: Serial.println(F("Type:             unknown family Dallas key")); break;             
  }
  keyType = keyUnknown;
  return true;
}

//************ Cyfral ***********************
unsigned long pulseACompA(bool pulse, byte Average = 80, unsigned long timeOut = 1500){  // pulse HIGH or LOW
  bool AcompState;
  unsigned long tEnd = micros() + timeOut;
  do {
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1 << ADSC));                                                      // Wait until the ADSC bit has been cleared
    if (ADCH > 200) return 0;
    if (ADCH > Average) AcompState = HIGH;                                            // читаем флаг компаратора
      else AcompState = LOW;
    if (AcompState == pulse) {
      tEnd = micros() + timeOut;
      do {
          ADCSRA |= (1<<ADSC);
          while(ADCSRA & (1 << ADSC));                                                // Wait until ADSC bit has been cleared
        if (ADCH > Average) AcompState = HIGH;                                        // читаем флаг компаратора
          else AcompState = LOW;
        if (AcompState != pulse) return (unsigned long)(micros() + timeOut - tEnd);  
      } while (micros() < tEnd);
      return 0;                                                                       // таймаут, импульс не вернулся обратно
    }             // end if
  } while (micros() < tEnd);
  return 0;
}

void ADCsetOn(){
  ADMUX = (ADMUX&0b11110000) | 0b0011 | (1<<ADLAR);// (1 << REFS0);                   // подключаем к AC Линию A3 ,  левое выравние, измерение до Vcc
  ADCSRB = (ADCSRB & 0b11111000) | (1<<ACME);                                         // источник перезапуска ADC FreeRun, включаем мультиплексор AC
  ADCSRA = (ADCSRA & 0b11111000) |0b011 | (1<<ADEN) | (1<<ADSC);// | (1<<ADATE);      // 0b011 делитель скорости ADC, // включаем ADC и запускаем ADC и autotriger ADC 
}

void ACsetOn(){
  ACSR |= 1<<ACBG;                                                                    // Подключаем ко входу Ain0 1.1V для Cyfral/Metacom
  ADCSRA &= ~(1<<ADEN);                                                               // выключаем ADC
  ADMUX = (ADMUX&0b11110000) | 0b0011;                                                // подключаем к AC Линию A3
  ADCSRB |= 1<<ACME;                                                                  // включаем мультиплексор AC
}

bool read_cyfral(byte* buf, byte CyfralPin){
  unsigned long ti; byte i=0, j = 0, k = 0;
  analogRead(iButtonPin);
  ADCsetOn(); 
  byte aver = calcAverage();
  unsigned long tEnd = millis() + 30;
  do{
    ti = pulseACompA(HIGH, aver);
    if ((ti == 0) || (ti > 260) || (ti < 10)) {i = 0; j=0; k = 0; continue;}
    if ((i < 3) && (ti > halfT)) {i = 0; j = 0; k = 0; continue;}      //контроль стартовой последовательности 0b0001
    if ((i == 3) && (ti < halfT)) continue;      
    if (ti > halfT) bitSet(buf[i >> 3], 7-j);
      else if (i > 3) k++; 
    if ((i > 3) && ((i-3)%4 == 0) ){        //начиная с 4-го бита проверяем количество нулей каждой строки из 4-и бит
      if (k != 1) {for (byte n = 0; n < (i >> 3)+2; n++) buf[n] = 0; i = 0; j = 0; k = 0; continue;}        //если нулей больше одной - начинаем сначала 
      k = 0; 
    }
    j++; if (j>7) j=0;
    i++;
  } while ((millis() < tEnd) && (i < 36));
  if (i < 36) return false;
  return true;
}

bool searchCyfral(){
  byte buf[8];
  for (byte i = 0; i < 8; i++) {addr[i] =0; buf[i] = 0;}
  if (!read_cyfral(addr, iButtonPin)) return false;
  if (!read_cyfral(buf, iButtonPin)) return false;
  for (byte i = 0; i < 8; i++) 
    if (addr[i] != buf[i]) return false;
  keyType = keyCyfral;
  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); Serial.print(":");
    keyID[i] = addr[i];                                         // копируем прочтенный код в ReadID
  }
  Serial.println(F("Type:             Cyfral "));
  return true;  
}

byte calcAverage(){
  unsigned int sum = 127; byte preADCH = 0, j = 0; 
  for (byte i = 0; i<255; i++) {
    ADCSRA |= (1<<ADSC);
    delayMicroseconds(10);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    sum += ADCH;
  }
  sum = sum >> 8;
  unsigned long tSt = micros();
  for (byte i = 0; i<255; i++) {
    delayMicroseconds(4);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    if (((ADCH > sum)&&(preADCH < sum)) | ((ADCH < sum)&&(preADCH > sum))) {
      j++;
      preADCH = ADCH;
    }
  }
  halfT = (byte)((micros() - tSt) / j);
  return (byte)sum;
}

bool read_metacom(byte* buf, byte MetacomPin){
  unsigned long ti; byte i = 0, j = 0, k = 0;
  analogRead(iButtonPin);
  ADCsetOn();
  byte aver = calcAverage();
  unsigned long tEnd = millis() + 30;
  do{
    ti = pulseACompA(LOW, aver);
    if ((ti == 0) || (ti > 500)) {i = 0; j=0; k = 0; continue;}
    if ((i == 0) && (ti+30 < (halfT<<1))) continue;      //вычисляем период;
    if ((i == 2) && (ti > halfT)) {i = 0; j = 0;  continue;}      //вычисляем период;
    if (((i == 1) || (i == 3)) && (ti < halfT)) {i = 0; j = 0; continue;}      //вычисляем период;
    if (ti < halfT) {   
      bitSet(buf[i >> 3], 7-j);
      if (i > 3) k++;                             // считаем кол-во единиц
    }
    if ((i > 3) && ((i-3)%8 == 0) ){        //начиная с 4-го бита проверяем контроль четности каждой строки из 8-и бит
      if (k & 1) { for (byte n = 0; n < (i >> 3)+1; n++) buf[n] = 0; i = 0; j = 0;  k = 0; continue;}              //если нечетно - начинаем сначала
      k = 0;
    }   
    j++; if (j>7) j=0;
    i++;
  }  while ((millis() < tEnd) && (i < 36));
  if (i < 36) return false;
  return true;
}

bool searchMetacom(){
  byte buf[8];
  for (byte i = 0; i < 8; i++) {addr[i] =0; buf[i] = 0;}
  if (!read_metacom(addr, iButtonPin)) return false;
  if (!read_metacom(buf, iButtonPin)) return false;
  for (byte i = 0; i < 8; i++) 
    if (addr[i] != buf[i]) return false; 
  keyType = keyMetacom;
  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); Serial.print(":");
    keyID[i] = addr[i];                               // копируем прочтенный код в ReadID
  }
  Serial.println(F("Type:             Metacom "));
  return true;  
}

/*void readAnalog(){
  byte buf[255][2];
  analogRead(iButtonPin);
  digitalWrite(iButtonPin, LOW); pinMode(iButtonPin, OUTPUT);  //отклчаем питание от ключа
  delay(1000);
  ADCsetOn();
  pinMode(iButtonPin, INPUT);                                   // включаем пиание Metacom 
  unsigned long ti = micros();
  for (byte i = 0; i < 255; i++){
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    buf[i][0] = ADCH;
    buf[i][1] = (byte) ((micros() - ti)>>1);
  }
  for (byte i = 0; i < 255; i++){
    Serial.print(buf[i][0]); Serial.print(" "); Serial.println(buf[i][1]);
  }
}

void resetMetacomCyfral(bool rsMc = true){

}
*/
//**********EM-Marine***************************
bool vertEvenCheck(byte* buf){        // проверка четности столбцов с данными
  byte k;
  k = 1&buf[1]>>6 + 1&buf[1]>>1 + 1&buf[2]>>4 + 1&buf[3]>>7 + 1&buf[3]>>2 + 1&buf[4]>>5 + 1&buf[4] + 1&buf[5]>>3 + 1&buf[6]>>6 + 1&buf[6]>>1 + 1&buf[7]>>4;
  if (k&1) return false;
  k = 1&buf[1]>>5 + 1&buf[1] + 1&buf[2]>>3 + 1&buf[3]>>6 + 1&buf[3]>>1 + 1&buf[4]>>4 + 1&buf[5]>>7 + 1&buf[5]>>2 + 1&buf[6]>>5 + 1&buf[6] + 1&buf[7]>>3;
  if (k&1) return false;
  k = 1&buf[1]>>4 + 1&buf[2]>>7 + 1&buf[2]>>2 + 1&buf[3]>>5 + 1&buf[3] + 1&buf[4]>>3 + 1&buf[5]>>6 + 1&buf[5]>>1 + 1&buf[6]>>4 + 1&buf[7]>>7 + 1&buf[7]>>2;
  if (k&1) return false;
  k = 1&buf[1]>>3 + 1&buf[2]>>6 + 1&buf[2]>>1 + 1&buf[3]>>4 + 1&buf[4]>>7 + 1&buf[4]>>2 + 1&buf[5]>>5 + 1&buf[5] + 1&buf[6]>>3 + 1&buf[7]>>6 + 1&buf[7]>>1;
  if (k&1) return false;
  if (1&buf[7]) return false;
  //номер ключа, который написан на корпусе
  rfidData[0] = (0b01111000&buf[1])<<1 | (0b11&buf[1])<<2 | buf[2]>>6;
  rfidData[1] = (0b00011110&buf[2])<<3 | buf[3]>>4;
  rfidData[2] = buf[3]<<5 | (0b10000000&buf[4])>>3 | (0b00111100&buf[4])>>2;
  rfidData[3] = buf[4]<<7 | (0b11100000&buf[5])>>1 | 0b1111&buf[5];
  rfidData[4] = (0b01111000&buf[6])<<1 | (0b11&buf[6])<<2 | buf[7]>>6;
  return true;
}

byte ttAComp(unsigned long timeOut = 7000){  // pulse 0 or 1 or -1 if timeout
  byte AcompState, AcompInitState;
  unsigned long tEnd = micros() + timeOut;
  AcompInitState = (ACSR >> ACO)&1;               // читаем флаг компаратора
  do {
    AcompState = (ACSR >> ACO)&1;                 // читаем флаг компаратора
    if (AcompState != AcompInitState) {
      delayMicroseconds(1000/(rfidBitRate*4));    // 1/4 Period on 2 kBps = 125 mks 
      AcompState = (ACSR >> ACO)&1;               // читаем флаг компаратора      
      delayMicroseconds(1000/(rfidBitRate*2));    // 1/2 Period on 2 kBps = 250 mks 
      return AcompState;  
    }
  } while (micros() < tEnd);
  return 2;                                             //таймаут, компаратор не сменил состояние
}

bool readEM_Marie(byte* buf){
  unsigned long tEnd = millis() + 50;
  byte ti; byte j = 7, k=0;
  for (int i = 0; i<64; i++){    // читаем 64 bit
    ti = ttAComp();
    if (ti == 2)  break;         //timeout
    if ( ( ti == 0 ) && ( i < 9)) {  // если не находим 9 стартовых единиц - начинаем сначала
      if (millis() > tEnd) { ti=2; break;}  //timeout
      i = -1; j=7; continue;
    }
    if ((i > 8) && (i < 59)){     //начиная с 9-го бита проверяем контроль четности каждой строки
      if (ti) k++;                // считаем кол-во единиц
      if ( (i-9)%5 == 4 ){        // конец строки с данными из 5-и бит, 
        if (k & 1) {              //если нечетно - начинаем сначала
          i = -1; j = 7; k = 0; continue; 
        }
        k = 0;
      }
    }
    if (ti) bitSet(buf[i >> 3], j);
      else bitClear(buf[i >> 3], j);
    if (j==0) j=8; j--;
  }
  if (ti == 2) return false;         //timeout
  return vertEvenCheck(buf);
}

void rfidACsetOn(){
  //включаем генератор 125кГц
  pinMode(FreqGen, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  //Вкючаем режим Toggle on Compare Match на COM2A (pin 11) и счет таймера2 до OCR2A
  TCCR2B = _BV(WGM22) | _BV(CS20);                                // Задаем делитель для таймера2 = 1 (16 мГц)
  OCR2A = 63;                                                    // 63 тактов на период. Частота на COM2A (pin 11) 16000/64/2 = 125 кГц, Скважнось COM2A в этом режиме всегда 50% 
  OCR2B = 31;                                                     // Скважность COM2B 32/64 = 50%  Частота на COM2A (pin 3) 16000/64 = 250 кГц
  // включаем компаратор
  ADCSRB &= ~(1<<ACME);           // отключаем мультиплексор AC
  ACSR &= ~(1<<ACBG);             // отключаем от входа Ain0 1.1V
}

bool searchEM_Marine( bool copyKey = true){
///  byte gr = digitalRead(G_Led);
///  bool rez = false;
digitalWrite(G_Led, LOW);
  rfidACsetOn();            // включаем генератор 125кГц и компаратор
  delay(6);                //13 мс длятся переходные прцессы детектора 
  if (!readEM_Marie(addr)) {
////    if (!copyKey) TCCR2A &=0b00111111;              //Оключить ШИМ COM2A (pin 11)
///    digitalWrite(G_Led, gr);
digitalWrite(G_Led, HIGH);
    return false;///rez;
  }
///  rez = true;
  keyType = keyEM_Marine;
  for (byte i = 0; i<8; i++){
    if (copyKey) keyID[i] = addr [i];
    Serial.print(addr[i], HEX); Serial.print(":");
  }
  Serial.print(F(" ( id "));
  Serial.print(rfidData[0]); Serial.print(" key ");
  unsigned long keyNum = (unsigned long)rfidData[1]<<24 | (unsigned long)rfidData[2]<<16 | (unsigned long)rfidData[3]<<8 | (unsigned long)rfidData[4];
  Serial.print(keyNum);
  Serial.println(F(") Type: EM-Marie "));
////  if (!copyKey) TCCR2A &=0b00111111;              //Оключить ШИМ COM2A (pin 11)
///  digitalWrite(G_Led, gr);
digitalWrite(G_Led, HIGH);
  return true;///rez;
}

void TxBitRfid(byte data){
  if (data & 1) delayMicroseconds(54*8); //1 48-56-64 Tc
    else delayMicroseconds(24*8); //0 16-24-32 Tc
  rfidGap(19*8);                  //write gap 8-10-20 Tc
}

void rfidGap(unsigned int tm){
  TCCR2A &=0b00111111;                //Оключить ШИМ COM2A 
  delayMicroseconds(tm);
  TCCR2A |= _BV(COM2A0);              // Включить ШИМ COM2A (pin 11)      
}

bool T5557_blockRead(byte* buf){
  byte ti; byte j = 7, k=0;
  for (int i = 0; i<33; i++){                           // читаем стартовый 0 и 32 значащих bit
    ti = ttAComp(2000);
    if (ti == 2)  break;                                //timeout
    if ( ( ti == 1 ) && ( i == 0)) { ti=2; break; }     // если не находим стартовый 0 - это ошибка
    if (i > 0){                                         //начиная с 1-го бита пишем в буфер
      if (ti) bitSet(buf[(i-1) >> 3], j);
        else bitClear(buf[(i-1) >> 3], j);
      if (j==0) j=8; j--;
    }
  }
  if (ti == 2) return false;                           //timeout
  return true;
}

bool sendOpT5557(byte opCode, unsigned long password = 0, byte lockBit = 0, unsigned long data = 0, byte blokAddr = 1){
  TxBitRfid(opCode >> 1); TxBitRfid(opCode & 1); // передаем код операции 10
  if (opCode == 0b00) return true;
  // password
  TxBitRfid(lockBit & 1);               // lockbit 0
  if (data != 0){
    for (byte i = 0; i<32; i++) {
      TxBitRfid((data>>(31-i)) & 1);
    }
  }
  TxBitRfid(blokAddr>>2); TxBitRfid(blokAddr>>1); TxBitRfid(blokAddr & 1);      // адрес блока для записи
  delay(4);                                                                     // ждем пока пишутся данные
  return true;
}

bool write2rfidT5557(byte* buf){
  bool result; unsigned long data32;
  delay(6);
  for (byte k = 0; k<2; k++){                                       // send key data
    data32 = (unsigned long)buf[0 + (k<<2)]<<24 | (unsigned long)buf[1 + (k<<2)]<<16 | (unsigned long)buf[2 + (k<<2)]<<8 | (unsigned long)buf[3 + (k<<2)];
    rfidGap(30 * 8);                                                 //start gap 8-15-50 Tc
    sendOpT5557(0b10, 0, 0, data32, k+1);                            //передаем 32 бита ключа в blok k
    Serial.print('*'); delay(6);
  }
  delay(6);
  rfidGap(30 * 8);                  //start gap
  sendOpT5557(0b00);  //RESET
  delay(4);
  result = readEM_Marie(addr);
////  TCCR2A &=0b00111111;              //Оключить ШИМ COM2A (pin 11)
  for (byte i = 0; i < 8; i++)
    if (addr[i] != keyID[i]) { result = false; break; }
  if (!result){
    Serial.println("\ncopy failed");
    Serial.println(F("-----------------------"));
    LCD_printError(F("ошибка копир."));
    Sd_ErrorBeep();
  } else {
    Serial.println(F(" was copied successesfully"));
    Serial.println(F("-----------------------"));
    LCD_printError(F("ключ скопирован"), false);
    Sd_ReadOK();
    delay(2000);
  }
  digitalWrite(R_Led, HIGH);
  return result;  
}

emRWType getRfidRWtype(){
  unsigned long data32, data33; byte buf[4] = {0, 0, 0, 0}; 
////  rfidACsetOn();                // включаем генератор 125кГц и компаратор
  delay(13);                    //13 мс длятся переходные процессы детектора
  rfidGap(30 * 8);              //start gap
  sendOpT5557(0b11, 0, 0, 0, 1); //переходим в режим чтения Vendor ID 
  if (!T5557_blockRead(buf)) return rwUnknown; 
  data32 = (unsigned long)buf[0]<<24 | (unsigned long)buf[1]<<16 | (unsigned long)buf[2]<<8 | (unsigned long)buf[3];
  delay(4);
  rfidGap(20 * 8);          //write gap  
  data33 = 0b00000000000101001000000001000000 | (rfidUsePWD << 4);   //конфиг регистр 0b0000 0000 0001 0100 1000 0000 0100 0000
  sendOpT5557(0b10, 0, 0, data33, 0);   //передаем конфиг регистр
  delay(4);
  rfidGap(30 * 8);          //start gap
  sendOpT5557(0b11, 0, 0, 0, 1); //переходим в режим чтения Vendor ID 
  if (!T5557_blockRead(buf)) return rwUnknown; 
  data33 = (unsigned long)buf[0]<<24 | (unsigned long)buf[1]<<16 | (unsigned long)buf[2]<<8 | (unsigned long)buf[3];
  sendOpT5557(0b00, 0, 0, 0, 0);  // send Reset
  delay(6);
  if (data32 != data33) return rwUnknown;    
  Serial.print(F(" The rfid RW-key is T5557. Vendor ID is "));
  Serial.println(data32, HEX);
  return T5557;
}

bool write2rfid(){
  bool Check = true;
  if (searchEM_Marine(false)) {
    for (byte i = 0; i < 8; i++)
      if (addr[i] != keyID[i]) { Check = false; break; }  // сравниваем код для записи с тем, что уже записано в ключе.
    if (Check) {                                          // если коды совпадают, ничего писать не нужно
      digitalWrite(R_Led, LOW); 
      Serial.println(F(" it is the same key. Writing in not needed."));
      LCD_printError(F("уже в памяти"));
      Sd_ErrorBeep();
      digitalWrite(R_Led, HIGH);
      delay(1000);
      return false;
    }
  }
  emRWType rwType = getRfidRWtype(); // определяем тип T5557 (T5577) или EM4305
  if (rwType != rwUnknown) { Serial.println(F("-----------------------")); Serial.print(F("Burning rfid ID: ")); }
  switch (rwType){
    case T5557: return write2rfidT5557(keyID); break;                    //пишем T5557
    //case EM4305: return write2rfidEM4305(keyID); break;                  //пишем EM4305
    case rwUnknown: break;
///  }
  }
  return false;
}

void SendEM_Marine(byte* buf){
  TCCR2A &=0b00111111; // отключаем шим
  digitalWrite(FreqGen, LOW);
  //FF:A9:8A:A4:87:78:98:6A
  delay(20);
  for (byte k = 0; k<10; k++){ //Зачем???
    for (byte i = 0; i<8; i++){
      for (byte j = 0; j<8; j++){
        if (1 & (buf[i]>>(7-j))) {
          pinMode(FreqGen, INPUT);
          delayMicroseconds(250);
          pinMode(FreqGen, OUTPUT);
        } else {
          pinMode(FreqGen, OUTPUT);
          delayMicroseconds(250);
          pinMode(FreqGen, INPUT);
        }
        delayMicroseconds(250);
      }
    }
   // delay(1);
  }
          pinMode(FreqGen, OUTPUT);//Возврат в исходное на выход
}

void SendDallas(byte* buf){
/*  iBtnEmul.init(buf);
  //iBtnEmul.waitForRequest(false);
  unsigned long tStart = millis();
  do {
    if (!iBtnEmul.waitReset(10) ) continue;
    if (!iBtnEmul.presence() ) continue;
    if (iBtnEmul.recvAndProcessCmd() ) break;
  } while (millis() < 200 + tStart);  */
}

void BM_SendKey(byte* buf){
  switch (keyType){
    case keyEM_Marine: SendEM_Marine(buf); break;
    default: SendDallas(buf); break;
  }
}

// обработка команд
unsigned long stTimer = millis();
void loop() {
  char echo = Serial.read();
  if ( (enc1.held(2) || echo == 'e') ) {                                // стираем EEPROM при удержании кнопки после двойного нажатия
    //myOLED.print(F("EEPROM cleared success!"), 0, 0);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("EEPROM очищен!"));
    Serial.println(F("EEPROM cleared"));
    EEPROM.update(0, 0); EEPROM.update(1, 0);
    EEPROM_key_count = 0; EEPROM_key_index = 0;
    Sd_ReadOK();
//    myOLED.update();
  }
/*  if (echo == 'r') readAnalog();
  if (echo == 'm') resetMetacomCyfral(true);
  if (echo == 'c') resetMetacomCyfral(false);*/
  if ((echo == 't') || enc1.click()) {                                   // переключаель режимов работы устройства
    switch (copierMode){
      case md_empty: Sd_ErrorBeep(); break;
      case md_read: copierMode = md_write; clearLed(); lcd_print_mode="|W"; EEPROM_get_key(EEPROM_key_index, keyID); LCD_printKey(keyID); break;
      case md_write: copierMode = md_blueMode; clearLed(); lcd_print_mode="|B"; EEPROM_get_key(EEPROM_key_index, keyID); LCD_printKey(keyID); break;
      case md_blueMode: copierMode = md_write_uni; clearLed(); lcd_print_mode="|U"; LCD_printKey(UniKey[UNI_key_index][8], 4); break;
      case md_write_uni: copierMode = md_read; clearLed(); lcd_print_mode="|R"; EEPROM_get_key(EEPROM_key_index, keyID); LCD_printKey(keyID); break;
    }
    //LCD_printKey(keyID);
    Serial.print(F("Mode: ")); Serial.println(copierMode);
    Sd_WriteStep();
    scroll_flag = 0;                                                      // восстанавливаем возможность вывода ID ключа
  }
  if ( echo == 'l' || enc1.left() ) {          //при повороте энкодера листаем ключи из eeprom
    if ( (copierMode != md_write_uni) && (EEPROM_key_count > 0) ) {
      if (--EEPROM_key_index < 1) EEPROM_key_index = EEPROM_key_count;     // уменьшаем индекс ключа
      EEPROM_get_key(EEPROM_key_index, keyID);
      Serial.print("Selected key: "); for (byte i = 0; i<8; i++){Serial.print(keyID[i], HEX); if (i<7) Serial.print(":");} Serial.println();
      LCD_printKey(keyID);
      Sd_WriteStep();
      scroll_flag = 0;                                                     // восстанавливаем возможность вывода ID ключа
    }
    else if (copierMode == md_write_uni) {
      if (--UNI_key_index < 0) UNI_key_index = UNI_key_count;
      Serial.print("Selected key: "); for (byte i = 0; i<8; i++){Serial.print(UniKey[UNI_key_index][i], HEX); if (i<7) Serial.print(":");} Serial.println();
      LCD_printKey(UniKey[UNI_key_index][8], 4);
      Sd_WriteStep();
      scroll_flag = 0;
    }
  }

  if ( echo == 'r' || enc1.right() ) {
    if ( (copierMode != md_write_uni) && (EEPROM_key_count > 0) ) {
      if (++EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;     // увеличиваем индекс ключа
      EEPROM_get_key(EEPROM_key_index, keyID);
      Serial.print("Selected key: "); for (byte i = 0; i<8; i++){Serial.print(keyID[i], HEX); if (i<7) Serial.print(":");} Serial.println();
      LCD_printKey(keyID);
      Sd_WriteStep();
      scroll_flag = 0;                                                     // восстанавливаем возможность вывода ID ключа        
    }
    else if (copierMode == md_write_uni) {
      if (++UNI_key_index > UNI_key_count) UNI_key_index = 0;
      Serial.print("Selected key: "); for (byte i = 0; i<8; i++){Serial.print(UniKey[UNI_key_index][i], HEX); if (i<7) Serial.print(":");} Serial.println();
      LCD_printKey(UniKey[UNI_key_index][8], 4);
      Sd_WriteStep();
      scroll_flag = 0;
    } 
  }

  if (enc1.turn() && (EEPROM_key_count == 0) && (copierMode != md_write_uni) ) {     // выводим начальный экран в режиме чтения после очистки памяти и поворота энкодера
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Приложите ключ.."));
    Sd_WriteStep();
    copierMode = md_read;
    lcd_print_mode="|R";
    clearLed();
  }
  if ( enc1.leftH() ) {                                                    // выводим ID ключа при нажатии и повороте энкодера налево
    if ( (copierMode != md_write_uni) && (EEPROM_key_count > 0) && (scroll_flag == 0) ) {
      scroll_flag = 1;                                                     // Принуждаем одиночное срабатываение. Т.к. функция 'rightH' фиксирует каждый щелчок поворота как отдельное прерывание
      String sq = "";
      for (byte i = 0; i < 8; i++) sq += String(keyID[i], HEX) +":";       // добавляем разелитель (:) в ID ключа
      sq.remove(sq.length()-1);                                            // удаляем лишний разделитель в конце строки
      scrollMessage(1, sq, 250);                                       // выводим ID ключа бегущей строкой во вторм ряду со скоростью 250мс
      EEPROM_get_key(EEPROM_key_index, keyID);
      LCD_printKey(keyID);                                                 // возвращаемся к исходному экрану
      Sd_WriteStep();
    }
    else if ( (copierMode == md_write_uni) && (scroll_flag == 0) ) {
      scroll_flag = 1;                                                     
      String sq = "";
      for (byte i = 0; i < 8; i++) sq += String(UniKey[UNI_key_index][i], HEX) +":";       
      sq.remove(sq.length()-1);                                            
      scrollMessage(1, sq, 250);                                       
      LCD_printKey(UniKey[UNI_key_index][8], 4);                            
      Sd_WriteStep();
    }
  }

  if ( enc1.rightH() && (copierMode == md_write_uni) ) {         // отобразим тип универсального ключа
    if ( rightH_flag == 0 ) {                                   // последовательно переключаемся между выводом имени и типа, если энкодер продолжает вращаться
      keyType = getKeyType(UniKey[UNI_key_index]);
      lcd.setCursor(0,1);
      lcd.print("                ");
      lcd.setCursor(0,1);
      String sr = "";
      switch (keyType){
        case keyDallas: sr += "Dallas"; break;      
        case keyCyfral: sr += "Cyfral";  break;  
        case keyMetacom: sr += "Metakom"; break;             
        case keyEM_Marine: sr += "EM_Marine"; break;
        case keyUnknown: sr += "Unknown"; break;
      }
      lcd.print("тип: " + String(sr));
      rightH_flag = 1;
    }
    else { lcd.setCursor(0,1); lcd.print("                "); lcd.setCursor(0,1); lcd.print("имя: " + NameKey[UNI_key_index]); rightH_flag = 0; }
  }

  if ((copierMode != md_empty) && (enc1.held(0) || echo == 's') && (copierMode != md_write_uni) ) {       // Если зажать кнопкку - ключ сохранися в EEPROM
    if (EPPROM_AddKey(keyID)) {
      LCD_printError(F("ключ сохранен"), false);
      Sd_ReadOK();
      delay(1000); 
    }
      else Sd_ErrorBeep();
    LCD_printKey(keyID);  
  }   
  if (millis() - stTimer < 100) return; //задержка в 100 мс
  stTimer = millis();
  switch (copierMode){
    case md_empty: case md_read: 
      if (searchCyfral() || searchMetacom() || searchEM_Marine() || searchIbutton() ){     // запускаем поиск cyfral, затем поиск EM_Marine, затем поиск dallas
        //keyID[0] = 0xFF; keyID[1] = 0xA9; keyID[2] =  0x8A; keyID[3] = 0xA4; keyID[4] = 0x87; keyID[5] = 0x78; keyID[6] = 0x98; keyID[7] = 0x6A;
        Sd_ReadOK();
        copierMode = md_read;
        digitalWrite(G_Led, HIGH);
        if (indxKeyInROM(keyID) == 0) LCD_printKey(keyID, 1);
          else LCD_printKey(keyID, 3);
        } 
      break;
    case md_write:
      if (keyType == keyEM_Marine) write2rfid();
        else write2iBtn(); 
      break;
    case md_blueMode: 
      BM_SendKey(keyID);
      break;
    case md_write_uni:
      for(byte i = 0; i < 8; i++) keyID_temp[i]=keyID[i];            // временно перемещаем ключ для режимов R|B чтобы не переписывать исходную функцию записи ключа
      for(byte i = 0; i < 8; i++) keyID[i]=UniKey[UNI_key_index][i];
      if (keyType == keyEM_Marine) write2rfid();
        else write2iBtn(); 
      for(int i=0; i<7; i++) keyID[i]=keyID_temp[i];
      break;
  } //end switch
}

//***************** звуки****************
void Sd_ReadOK() {  // звук ОК
  for (int i=400; i<6000; i=i*1.5) { tone(speakerPin, i); delay(20); }
  noTone(speakerPin);
}

void Sd_WriteStep(){  // звук "очередной шаг"
  for (int i=2500; i<6000; i=i*1.5) { tone(speakerPin, i); delay(10); }
  noTone(speakerPin);
}

void Sd_ErrorBeep() {  // звук "ERROR"
  for (int j=0; j <3; j++){
    for (int i=1000; i<2000; i=i*1.1) { tone(speakerPin, i); delay(10); }
    delay(50);
    for (int i=1000; i>500; i=i*1.9) { tone(speakerPin, i); delay(10); }
    delay(50);
  }
  noTone(speakerPin);
}

void Sd_StartOK(){   // звук "Успешное включение"
  tone(speakerPin, NOTE_A7); delay(100);
  tone(speakerPin, NOTE_G7); delay(100);
  tone(speakerPin, NOTE_E7); delay(100); 
  tone(speakerPin, NOTE_C7); delay(100);  
  tone(speakerPin, NOTE_D7); delay(100); 
  tone(speakerPin, NOTE_B7); delay(100); 
  tone(speakerPin, NOTE_F7); delay(100); 
  tone(speakerPin, NOTE_C7); delay(100);
  noTone(speakerPin); 
}
