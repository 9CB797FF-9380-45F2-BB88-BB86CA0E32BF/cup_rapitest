#include <string.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
#include <softPwm.h>
#include <time.h>

#define CS_MCP3208 8  
#define SPI_CHANNEL 0
#define SPI_SPEED 1000000 //1Mhz
#define MAXTIMINGS 85
#define HUMAN 0
#define COLLISION 3
#define LIGHT 0
#define LEDP 24
#define LEDR 7
#define LEDG 8
#define LEDB 9

float h, t;
float humi = 15;
float h_arr[5];
float h_average = 0;
static int DHTPIN = 11;
static int dht22_dat[5] = {0,0,0,0,0};
int light_value = 0;
int now_color = 0; // 빨(0), 노(1), 초(2), 시안(3), 파(4)
int now_brightness = 0; // 0:30%, 1:65%, 2:100%
int user_setting_brightness = 0; // 0:30%, 1:65%, 2:100%
int mode = 0; // 수동(0), 자동(1)
clock_t last_detect_time;

static u_int8_t sizecvt(const int read) {
    if (read > 255 || read < 0) {
        printf("Invalid data from wiringPi library\n");
        exit(1);
    }
    return (u_int8_t) read;
}

static int read_dht22_dat() {
    u_int8_t laststate = HIGH;
    u_int8_t counter = 0;
    u_int8_t j = 0, i;
    dht22_dat[0] = dht22_dat[1] = dht22_dat[2] = dht22_dat[3] = dht22_dat[4] = 0;
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHTPIN, HIGH);
    delay(10);
    digitalWrite(DHTPIN, LOW);
    delay(18);
    digitalWrite(DHTPIN, HIGH);
    delayMicroseconds(40); 
    pinMode(DHTPIN, INPUT);
    for (i=0; i< MAXTIMINGS; i++) {
        counter = 0;
        while (sizecvt(digitalRead(DHTPIN)) == laststate) {
            counter++;
            delayMicroseconds(1);
            if (counter == 255) {
                break;
            }
        }
        laststate = sizecvt(digitalRead(DHTPIN));
        if (counter == 255) break;
        if ((i >= 4) && (i%2 == 0)) {
            dht22_dat[j/8] <<= 1;
            if (counter > 50)
                dht22_dat[j/8] |= 1;
            j++;
        }
    }
    if ((j >= 40) && (dht22_dat[4] == ((dht22_dat[0] + dht22_dat[1] + dht22_dat[2] + dht22_dat[3]) & 0xFF)) ) {
        h = (float)dht22_dat[0] * 256 + (float)dht22_dat[1];
        h /= 10;
        t = (float)(dht22_dat[2] & 0x7F)* 256 + (float)dht22_dat[3];
        t /= 10.0;
        if ((dht22_dat[2] & 0x80) != 0)  t *= -1;
        printf("습도 = %.2f %%, 온도 = %.2f *C \n", h, t );   
        return 1;
    }
    else {
        printf("Data not good, skip\n");
        return 0;
    }
}

// spi communication with Rpi and get sensor data
int read_mcp3208_adc(unsigned char adcChannel) {
    unsigned char buff[3];
    int adcValue = 0;
    buff[0] = 0x06 | ((adcChannel & 0x07) >> 2);
    buff[1] = ((adcChannel & 0x07) << 6);
    buff[2] = 0x00;
    digitalWrite(CS_MCP3208, 0);
    wiringPiSPIDataRW(SPI_CHANNEL, buff, 3);
    buff[1] = 0x0f & buff[1];
    adcValue = (buff[1] << 8 ) | buff[2];
    digitalWrite(CS_MCP3208, 1);
    return adcValue;
}

void sig_handler (int signo) {
    printf("process stop…\n");
    digitalWrite(LEDR, 0);
    digitalWrite(LEDB, 0);
    digitalWrite(LEDG, 0);
    digitalWrite(LEDP, 0); 
    exit(0);
}

void colorSet(int color, int brightness) {
    // color: 빨(0), 노(1), 초(2), 시안(3), 파(4)
    // brightness: 0:30%, 1:65%, 2:100%
    float power[3] = {0.3, 0.65, 1};
    int r = 0; 
    int g = 0;
    int b = 0;
    if(color == 0){
        r = 255;
        g = 0;
        b = 0;
    }
    else if(color == 1) {
        r = 255;
        g = 255;
        b = 0;
    }
    else if(color == 2) {
        r = 0;
        g = 255;
        b = 0;
    }
    else if(color == 3) {
        r = 0;
        g = 255;
        b = 255;
    }
    else if(color == 4) {
        r = 0;
        g = 0;
        b = 255;
    }

    softPwmWrite(LEDR, r * power[brightness]);
    softPwmWrite(LEDG, g * power[brightness]);
    softPwmWrite(LEDB, b * power[brightness]);
    printf("LED를 (색상:%d, 밝기:%d)로 설정합니다.\n", now_color, now_brightness);
}

void myInterrupt(void) {
    printf("사람이 감지되었습니다.\n");
    if(mode) {
        now_brightness = 255;
        printf("밝기를 100%%으로 지정합니다.\n");
    }
}

void myInterrupt2(void) {
    printf("근접센서가 감지되었습니다.\n");
    if((long)(clock()-last_detect_time) < 500) {
        mode = 1;
        printf("모드를 자동으로 설정합니다.\n");
    }
    else {
        last_detect_time = clock();
        mode = 0;
        printf("모드를 수동으로 설정합니다.\n");
        now_brightness = user_setting_brightness = (user_setting_brightness+1)%3;
        printf("밝기를 %d로 지정합니다.\n", now_brightness);
        colorSet(now_color, now_brightness);
    }
}

int main(void) {
    printf("프로그램 시작\n");

    last_detect_time = clock();

    if(wiringPiSetup() == -1)
    {
        fprintf(stdout, "Unable to start wiringPi :%s\n", strerror(errno));
        return 1;
    }
   
    if(wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1)
    {
        fprintf(stdout, "wiringPiSPISetup Failed :%s\n", strerror(errno));
        return 1;
    }
   
    pinMode(CS_MCP3208, OUTPUT);
    pinMode(HUMAN, INPUT);
    pinMode(COLLISION, INPUT);
    pinMode(LEDP, OUTPUT);
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    softPwmCreate(LEDR, 0, 255);
    softPwmCreate(LEDG, 0, 255);
    softPwmCreate(LEDB, 0, 255);

    signal(SIGINT, (void *)sig_handler);

    if(wiringPiISR(HUMAN, INT_EDGE_RISING, &myInterrupt) == -1) {
        printf("Unable to setup ISR\n");
        exit(1);
    }

    if(wiringPiISR(COLLISION, INT_EDGE_FALLING, &myInterrupt2) == -1) {
        printf("Unable to setup ISR\n");
        exit(1);
    }

    digitalWrite(LEDP, HIGH);
    printf("LED 전원을 켭니다\n");

    colorSet(1, 2);

    while(1) {
        if((humi >= 10) && (humi <= 25)) {
            now_color = 0;
            printf("현재 색상을 빨강으로 지정합니다.\n");
        }
        else if((humi >= 26) && (humi <= 34)) {
            now_color = 1;
            printf("현재 색상을 노랑으로 지정합니다.\n");
        }
        else if((humi >= 35) && (humi <= 50)) {
            now_color = 2;
            printf("현재 색상을 초록으로 지정합니다.\n");
        }
        else if((humi >= 51) && (humi <= 64)) {
            now_color = 3;
            printf("현재 색상을 시안으로 지정합니다.\n");
        }
        else if((humi >= 65) && (humi <= 80)) {
            now_color = 4;
            printf("현재 색상을 파랑으로 지정합니다.\n");
        }

        for(int i = 0; i < 5; i++) {
            read_dht22_dat();
            h_arr[i] = h;
            printf("평균계산을 위한 센서읽기 %d 회\n", i);
            delay(60000); // 1분
        }

        h_average = 0;
        for(int i = 0; i < 5; i++) {
            h_average += h_arr[i];
        }

        humi = h_average / 5;
        printf("습도 평균: %f\n", humi);

        if(mode) {
            light_value = read_mcp3208_adc(LIGHT);
            printf("조도: %d\n", light_value);
            if(light_value <= 500) {
                now_brightness = 2;
                printf("밝기를 100%%로 지정합니다.\n");
            }
            if(light_value <= 1000) {
                now_brightness = 1;
                printf("밝기를 65%%로 지정합니다.\n");
            }
            if(light_value <= 1500) {
                now_brightness = 0;
                printf("밝기를 30%%로 지정합니다.\n");
            }
        }
        colorSet(now_color, now_brightness);

    }
    return 0;
}