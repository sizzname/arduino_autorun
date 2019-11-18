/*
  Автозапуск двигателя на Arduino Nano.
  
  Позволяет запустить двигатель по сигналу с дополнительного канала автосигнализации или звонком с телефона.

  Особенность и отличие от встречавшихся мне других проектов - непрерывно опрашиваются все датчики
  и в случае срабатывания какого-либо, процесс автозапуска двигателя прерывается, а если он уже заведён - то глушится.
  Особенно актуально это для датчика скорости, чтобы избежать движения автомобиля, если вдруг он стоит на скорости.
  То есть если во время процедуры автозапуска автомобиль начинает двигаться - то двигатель сразу глушится и автомобиль останавливается.

  Автозапуск возможен только при поднятом ручнике.

  Работа двигателя отслеживается по сигналу тахометра.

  Сигнал для нейтрального положения ручки КПП есть, как сделать датчик идеи есть, но пока так и не релизовано, да и как показала прошедшая зима, не очень и нужно.
  Дачтика скорости, переключения кпп и раздатки в нейтральное положение хватает, но лучше бы перестраховаться.

  Оповещение о начале работы автозапуска и прекращению приходит по смс.
  
  Телефон для управления (с которого принимаются команды и на который отправляются смс) указывается жёстко в программе.

  После дозвона нужно набрать код в тоновом режиме:
  123 - запустить двигатель
  777 - перезагрузить модем
  0 - заглушить двигатель
  # - завершить звонок
  
  НОМЕР ТЕЛЕФОНА НУЖНО УКАЗАТЬ НИЖЕ.

  Для автозапуска делается несколько попыток длительностью по 12 секунд (настраивается переменной).

  Проект делался для автомобиля Chevrolet Niva.

  Датчик скорости повешен на прерывание - обработка его сигнала в приоритете.

  В планах: 
  - датчик  нейтрали
  - автозапуск по температуре окружающей среды
  - включение кондиционера при высокой температуре

  created 2019
  by Denis B. Anikin
  
  https://denisanikin.ru/9150-autostart-chevrolet-niva


  За вдохновение спасибо:
  http://compcar.ru/forum/showthread.php?t=9812
  http://arduino.ru/forum/proekty/avtozapusk-pervyi-proekt
  https://www.drive2.ru/l/7715112/
  http://arduinoprom.ru/shemotehnika/261-avtozapusk-dvigatelja-na-arduino.html
  
  
  
*/

#include <SoftwareSerial.h>                     // Библиотека програмной реализации обмена по UART-протоколу
SoftwareSerial SIM800(5, 4);                    // RX, TX

const String phone = "+79ХХХХХХХХХ"; // Телефон для управления

const byte speedIn =             2; // сигнал с датчика скорости D2
const byte tahoIn =              3; // сигнал тахометра D3
const byte handBrakeIn =        14; // ручной тормоз  A0
const byte neutralGearIn =      15; // датчик нейтрали A1
const byte startSIn =           16; // сигнал автозапуска от сигнализации A2

const byte immoOut =           12; // обход иммобилизатора D12
const byte beeperOut =         11; // пищалка D11
const byte powerOut =          10; // зажигание D10
const byte starterOut =         9; // стартер D9
const byte fanOut =             8; // вентилятор отопителя (для отопления или кондиционирования, режим выставляется заранее, тот, с которым приехал) D8
const byte autostartModeOut =   7; // LED - сигнализация режима автозапуска D7
const byte signalOut =          6; // выход на реле блокировки сигнализации, включение его на время запуска двигателя D6

const byte handBrakeSignal   =     1; // сигнал, если ручник поднят
const byte neutralGearSignal =     0; // сигнал, если включена нейтралка

const byte maxTryesCount     =     3; // максимальное число попыток запуска
const byte powerDelayTime    =    30; // задержка включения питания (перерыв между попытками), секунд 
const byte starterDelayTime  =     5; // задержка включения стартера, секунд
const byte maxStartTime      =    12; // время работы стартера, секунд
const byte fanDelayTime      =    10; // задержка включения вентилятора отопителя, секунд
const word maxRunTime        =   900; // время работы двигателя после запуска, секунд (600 секунд - 10 минут, 1200 секунд - 20 минут)

volatile bool engineRuning   = false; // состояние двигателя, выключен / работает
bool          needStart      = false; // наличие запроса на запуск двигателя
bool          autostartMode  = false; // режим автозапуска, нет / да
bool          startMode      = false; // режим попытки запуска двигателя, нет / да
bool          powerOn        = false; // подали питание
volatile bool starterOn      = false; // включили стартер
volatile bool fanOn          = false; // включили вентилятор печки
unsigned long timeStart      = 0;     // время запуска двигателя
unsigned long tryStartTime   = 0;     // время начала очередной попытки запуска двигателя
byte          tryesCount     = 0;     // число попыток запуска
byte          tryCurrent     = 0;     // текущая попытка запуска
word          tahoSignal     = 0;     // сигнал тахометра
byte          blinkCount     = 0;
bool          ledOn          = false;
word          blinkLength    = 500;   // длительность сигнала светодиода и бипера
word          speedSignal    = 0;
String        pin            = "";    // строковая переменная набираемого DTMF пинкода


void setup() {
    pinMode(handBrakeIn, INPUT);
    pinMode(neutralGearIn, INPUT);
    pinMode(speedIn, INPUT);
    pinMode(tahoIn, INPUT);
    pinMode(startSIn, INPUT);
  
    pinMode(immoOut, OUTPUT);
    pinMode(powerOut, OUTPUT);
    pinMode(beeperOut, OUTPUT);
    pinMode(starterOut, OUTPUT);
    pinMode(fanOut, OUTPUT);
    pinMode(autostartModeOut, OUTPUT);
    pinMode(signalOut, OUTPUT);

    digitalWrite(immoOut, LOW);
    digitalWrite(powerOut, LOW);
    digitalWrite(beeperOut, LOW);
    digitalWrite(starterOut, LOW);
    digitalWrite(fanOut, LOW);
    digitalWrite(autostartModeOut, LOW);
    digitalWrite(signalOut, LOW);

    delay(2000);                    // задержка 10 секунд после включения питания
    Serial.begin(19200);            // Скорость обмена данными с компьютером
    SIM800.begin(19200);            // Скорость обмена данными с модемом 
    SIM800.println("AT");           // Отправили AT для настройки скорости обмена данными
    delay(500);
    //SIM800.println("ATD+79272206057;");
    SIM800.println("AT+CMGF=1;&W"); // Включаем текстовый режим SMS (Text mode) и сразу сохраняем значение (AT&W)!
    delay(1000);
    SIM800.println("AT+CMGDA=\"DEL ALL\"");
    delay(1000);
    sendMessage("System up", false);
    delay(1000);
    SIM800.println("AT+CFUN=1,1");  // перезагрузка модема

    blinkAndBeep(3, 100);

}

void loop() {

    unsigned long timeFromStart = 0;
    unsigned long currentMillis = millis(); // текущее время
    needStart = false;

    // получаем состояние двигателя
    tahoSignal = pulseIn(tahoIn, HIGH, 100000);
    engineRuning = (tahoSignal > 0 && tahoSignal < 18000);

    speedSignal = digitalRead(speedIn);
    //Serial.println(speedSignal);
    
    if (engineRuning) {
        if (timeStart == 0) {
            timeStart = currentMillis; // определяем момент запуска двигателя
        } else {
            timeFromStart = (currentMillis - timeStart) / 1000; // время работы двигателя в секундах
            // машина заведена, прошло время задержки для включения дополнительного оборудования, включаем его, если ещё не включено
            if (!fanOn && timeFromStart > fanDelayTime) {
                digitalWrite(fanOut, HIGH);
                fanOn = true;
            }
        }
    } else {
        if (fanOn) { // если двигатель заглушился, а доп оборудование было включено - выключаем его.
            digitalWrite(fanOut, LOW);
            fanOn = false;
        }
        
        
        timeStart = 0;
    }

    if (SIM800.available()) {
        processModem();    // если что-то пришло от SIM800 в Ардуино отправляем для разбора
    }
    if (Serial.available()) {
        processSerial();   // если что-то пришло от компа в Ардуино отправляем в SIM800
    }

   

    // режим автозапуска
    if (autostartMode) {

        // получаем сигнал с ручника и датчика нейтрали, 
        // а так же проверяем время прошедшее с момента запуска двигателя
        // если что-то не так - глушим всё
        if (
            digitalRead(startSIn) == 1 || // сигнал отключения
            digitalRead(handBrakeIn) != handBrakeSignal || // ручник
            digitalRead(neutralGearIn) != neutralGearSignal || // нейтралка
            tryesCount > maxTryesCount || // число попыток запуска
            timeFromStart > maxRunTime // время работы
          ) {
            
            autostartOff();
            delay (500);

            if (digitalRead(handBrakeIn) != handBrakeSignal) {
              //sendMessage("Ошибка: Сигнал ручника! Автозапуск отключен.");
              sendMessage("Handbrake", true);
              blinkAndBeep(3, 300);
            }

            if (digitalRead(neutralGearIn) != neutralGearSignal) {
              //sendMessage("Ошибка: Сигнал нейтрали! Автозапуск отключен.");
              sendMessage("Gearbox", true);
              blinkAndBeep(4, 300);
            }

            if (tryesCount > maxTryesCount) {
              //sendMessage("Ошибка: исчерпано число попыток. Автозапуск отключен.");
              sendMessage("Max tryes off", true);
              blinkAndBeep(5, 300);
            }

            if (timeFromStart > maxRunTime) {
              //sendMessage("Успешное завершение автозапуска. Время работы: ");
              sendMessage("Completed OK", false);
              blinkAndBeep(1, 1000);
            }
            
            return;
            
        } 
        
        if (startMode) {

            // запуск двигателя
           
            if (tryCurrent != tryesCount) { // новая попытка
              
                tryStartTime = currentMillis; // время начала очередной попытки
                tryesCount = tryCurrent;
                digitalWrite(starterOut, LOW);
                digitalWrite(immoOut, LOW);
                digitalWrite(beeperOut, LOW);
                digitalWrite(signalOut, LOW);
                digitalWrite(fanOut, LOW);
                digitalWrite(powerOut, LOW);
                powerOn = false;
                starterOn = false;
                fanOn = false;
                
                // если это первый запуск - включаем зажигание, чтобы аккумулятор начал работать
                if (tryCurrent == 1) {
                  digitalWrite(immoOut, HIGH);
                  delay(300);
                  digitalWrite(powerOut, HIGH);
                }
                
            } else {
              
                // сколько секунд прошло с начала процедуры
                unsigned int timePassed =  (currentMillis - tryStartTime) / 1000;
          
                // задержка автозапуска, включаем питание
                if (!powerOn && timePassed > powerDelayTime) {
                    digitalWrite(powerOut, LOW);
                    delay(500);
                    digitalWrite(immoOut, HIGH);
                    delay(300);
                    digitalWrite(powerOut, HIGH);
                    delay(500);
                    digitalWrite(signalOut, HIGH);
                    powerOn = true;
                }
          
                // задержка, включаем стартер если двигатель не запущен
                if (!engineRuning && !starterOn && timePassed > powerDelayTime + starterDelayTime) { //
                    digitalWrite(starterOut, HIGH);
                    starterOn = true;
                }
          
                // завелась, двигатель работает уже 2 секунды, выключаем стартер, выходим из режима запуска
                if (engineRuning && starterOn && timeFromStart > 0) {
                    digitalWrite(starterOut, LOW);
                    starterOn = false;
                    startMode = false;
                    sendMessage("Engine running", false);
                }
          
                // не завелась, а время вышло - выключаем стартер, запускаем новую попытку
                if (!engineRuning && starterOn && timePassed > powerDelayTime + starterDelayTime + maxStartTime) { // 
                  digitalWrite(starterOut, LOW);
                  starterOn = false;
                  tryCurrent = tryCurrent + 1;
                } 
              
            }
        
            // конец автозапук двигателя
            
        }

    } else {

        // получаем наличие сигналов на запуск двигателя
        if (digitalRead(startSIn) == 1  ) {
            needStart = true;
//        } else {
//            needStart = false;
        }
    
        if (!engineRuning) {  // если двигатель не заведён
            // сюда надо добавить проверки на возможность запуска
            if (needStart) {  // и есть сигнал на его запуск
                autostartMode = true;  // переходим в режим запуска двигателя
                startMode = true;      // и пробуем запустить двигатель
                tryesCount = 0;
                tryCurrent = 1;
                timeStart = 0;  
                digitalWrite(autostartModeOut, HIGH);
                sendMessage("AutoStart", false);
                delay(1000); // подождём, чтобы исчез сигнал, на случай, чтобы не повторять попытку неудачного запуска
                // вешаем прерывание на датчик скорости
                attachInterrupt(digitalPinToInterrupt(speedIn), speedChange, CHANGE);
            }
        }
    }

    if (blinkCount > 0) {
        makeSignal(currentMillis);
    }
    
    
}

void blinkAndBeep(int count, int signalLength){
    blinkCount = count;
    if (signalLength == 0) {
      blinkLength = 500;
    } else {
      blinkLength = signalLength;
    }
    ledOn = false;
    digitalWrite(beeperOut, LOW);
    digitalWrite(autostartModeOut, LOW);
    
}

void makeSignal(unsigned long currentMillis){
    int signalFull = blinkLength * 2;
    if (ledOn) {
        if (currentMillis % signalFull < blinkLength) {
          digitalWrite(autostartModeOut, LOW);
          digitalWrite(beeperOut, LOW);
          ledOn = false;
          blinkCount--;
        }
    } else {
        
        if (currentMillis % signalFull > blinkLength) {
          digitalWrite(autostartModeOut, HIGH);
          digitalWrite(beeperOut, HIGH);
          ledOn = true;
        }
    }
}

void speedChange(){
    if (starterOn || engineRuning) {
        autostartOff();
        //sendMessage("Ошибка: Началось движение! Автозапуск отключен:");
        sendMessage("Car moving", true);
        blinkAndBeep(2, 300);
    }
}

void autostartOff(){
    digitalWrite(starterOut, LOW);
    digitalWrite(signalOut, LOW);
    digitalWrite(immoOut, LOW);
    digitalWrite(powerOut, LOW);
    digitalWrite(beeperOut, LOW);
    digitalWrite(autostartModeOut, LOW);
    timeStart = 0;
    tryCurrent = 0;
    autostartMode = false;
    startMode = false;
    powerOn = false;
    starterOn = false;
    ledOn = false;
    detachInterrupt(digitalPinToInterrupt(speedIn));
}


void processSerial(){     // транслируем команды из порта в модем
  String at = "";   
  int k = 0;
  while (Serial.available()) {
    k = Serial.read();
    at += char(k);
    delay(1);
  }
  SIM800.println(at);
  at = "";
}   


void processModem (){     // читаем и обрабатываем сигналы от модема

  String at = "";
  int k = 0;
  while (SIM800.available()) {
    k = SIM800.read();
    at += char(k);
    delay(1);
  }
  Serial.println(at); // дублируем в серийный порт

   
  if (at.indexOf("+CLIP: \""+phone+"\",") > -1) { // звонок с телефона
    delay(200);
    SIM800.println("ATA");
    blinkAndBeep(1, 200);
/*
  } else if (at.indexOf("+CMTI:") > -1) { // входящая SMS
    int index = at.lastIndexOf(",");   // Находим последнюю запятую, перед индексом
    String result = at.substring(index + 1, at.length()); // Получаем индекс
    result.trim();                     // Убираем пробельные символы в начале/конце
    SIM800.println("AT+CMGR="+result); // Получить содержимое SMS

  } else if (at.indexOf("+CMGR:") > -1) { // обрабатываем SMS
    
    int index = at.indexOf("\r", 5);
    if (index > 0) {
        String mhdr = at.substring(at.indexOf("+CMGR:"), index);
        Serial.println(mhdr);
        if (mhdr.indexOf("\",\""+phone+"\",\"") > 0) { // проверяем номер телефона
          String msg = at.substring(index + 2, at.lastIndexOf("OK"));
          msg.trim();
          if (msg == "start") { // старт автозапуска
            needStart = true;
          } else if (msg == "reboot") {       // перезагрузка модема
            SIM800.println("ATH;+CFUN=1,1");
          } else {                            // любое другое сообщение выключает автозапуск
            autostartOff();
            
          }
        }
    }
  */  
  } else if (at.indexOf("+DTMF: ")  > -1) { 
    String key = at.substring(at.indexOf("")+9, at.indexOf("")+10);
    pin = pin + key;
    if (pin.indexOf("*") > -1 ) {
      pin= ""; 
    }
    
  } else if (at.indexOf("SMS Ready") > -1 || at.indexOf("NO CARRIER") > -1 ) {
    SIM800.println("AT+CLIP=1;+DDET=1"); // Активируем АОН и декодер DTMF
 
  }

  at = "";
  
  // обрабатываем DTFM код
  if (pin.indexOf("123") > -1 ) { // автозапуск
    pin= "";
    delay(300);
    SIM800.println("AT+VTS=\"1,2,3\"");
    delay(2000);
    SIM800.println("ATH");
    needStart = true;
    
  } else if (pin.indexOf("777") > -1 ) { // перезагрузить модем
    pin= "";
    SIM800.println("AT+CMGDA=\"DEL ALL\"");
    SIM800.println("ATH;+CFUN=1,1"); 
  
  } else if (pin.indexOf("0") > -1 ){ // остановить прогрев
    pin= "";
    autostartOff();
    SIM800.println("ATH");
    
  } else if (pin.indexOf("#") > -1 ) { // завершить звонок
    pin= "";
    SIM800.println("ATH");
  }


}


void sendMessage(String message, bool error){
  if (error) {
    message = "ERROR " + message;
  }
  Serial.println(message);
  delay(200);
  SIM800.println("AT+CMGS=\""+phone+"\"");  // Переходим в режим ввода текстового сообщения
  delay(200);
  SIM800.print(message);          // Вводим сообщение
  delay(200);
  SIM800.println((char)26);         // Уведомляем GSM-модуль об окончании ввода
  delay(200);
  SIM800.println(); 
  delay(2000);
}
