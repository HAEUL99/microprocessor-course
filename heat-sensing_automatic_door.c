#include <avr/io.h>
#define F_CPU 16000000UL
#define F_SCK 40000UL
#include <avr/interrupt.h>
#include <util/delay.h>
#define DISTANCE 200 // 20.0cm
#define TEMPERATURE 130 // 섭씨 13.0도
#define TRIG_ULTR 6 // 초음파 센서 TRIG 핀번호 설정
#define ECHO_ULTR 7 // 초음파 센서 ECHO 핀번호 설정
#define ATS75_ADDR 0x98 // 0b10011000, 7비트를 1비트 left shift
#define ATS75_CONFIG_REG 1
#define ATS75_TEMP_REG 0
#define MOTOR_MOVE 0x80 // 모터 이동
#define MOTOR_STOP 0x00// 모터 정지
int get_dist(void);
int get_temp(void);
void move_motor(int distance, int temperature);
int display_fnd_temp(int value);
void init(void);
void running(void);
void init_motor_port(void);
void init_twi_port(void);
void write_twi(char reg, char data);
int read_twi(char reg);
int main(void)
{
    init(); // 핀번호 및 ic2 통신 초기화
    running(); // 동작
}
void init(void)
{
    init_motor_port(); // 모터 핀번호 초기화
    init_twi_port(); // ic2 통신 초기화
}
void running(void)
{
    int distance;
    int temperature;
    while(1)
    {
        distance = get_dist();
        temperature = get_temp();
        sei();
        move_motor(distance, temperature);
    }
}
void init_motor_port()
{
    DDRB = 0xff;
    DDRC = 0xff;
    DDRG = 0x0f;
    DDRF = ((DDRF|(1<<TRIG_ULTR)) & ~(1<<ECHO_ULTR)); //TRIG, ECHO 핀 설정
}
void init_twi_port()
{
    DDRC = 0xff;
    DDRG = 0xff;
    PORTD = 3; // SCL & SCK Pull Up
    SFIOR &= ~(1<<PUD); // PUD = 0 : Pull Up Disable
    TWBR = (F_CPU/F_SCK - 16) / 2; // 공식 참조
    TWSR = TWSR & 0xfc;
    write_twi(ATS75_CONFIG_REG, 0x00);
    _delay_ms(100);
}
/*
aTS75에서 온도값을 read/write 작동 방법
aTS75의 configuration register를 적합한 값으로 write(9 bit 모드, normal 모드 선택, internal
address & register의 2바이트 데이터 전송)
aTS75의 command register가 temperature register를 포인팅하도록 세트한 후 2바이트
온도값 읽기
*/
void write_twi(char reg, char data)
{
    TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN);// START 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08) ;
    TWDR = ATS75_ADDR | 0; // SLA+W 준비, W=0
    TWCR = (1 << TWINT) | (1 << TWEN); // SLA+W 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18) ;
    TWDR = reg; // aTS75 Reg 값 준비
    TWCR = (1 << TWINT) | (1 << TWEN); // aTS75 Reg 값 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28) ;
    TWDR = data; // DATA 준비
    TWCR = (1 << TWINT) | (1 << TWEN); // DATA 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28) ;
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}
int read_twi(char reg)
{
    char high_byte, low_byte;
    TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN); // START 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08) ;
    TWDR = ATS75_ADDR | 0; // SLA+W 준비, W=0
    TWCR = (1 << TWINT) | (1 << TWEN); // SLA+W 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18) ;
    TWDR = reg; // aTS75 Reg 값 준비
    TWCR = (1 << TWINT) | (1 << TWEN); // aTS75 Reg 값 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28) ;
    TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN); // RESTART 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x10) ;
    TWDR = ATS75_ADDR | 1; // SLA+R 준비, R=1
    TWCR = (1 << TWINT) | (1 << TWEN); // SLA+R 전송
    while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x40) ;
    TWCR = (1 << TWINT) | (1 << TWEN | 1 << TWEA); // 1st DATA 준비
    while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x50);
    high_byte = TWDR; //1st DATA 수신
    TWCR = (1 << TWINT) | (1 << TWEN); // 2nd DATA 준비
    while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x58);
    low_byte = TWDR; // 2nd DATA 수신
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP 전송
    return((high_byte<<8) | low_byte); // 수신 DATA 리턴
}
// 초음파 센서: Port F - Trig: 6, Echo: 7
int get_dist(void)
{
    int distance;
    TCCR1B = 0x02;
    PORTF &= ~(1<<TRIG_ULTR); // TRIG = LOW
    _delay_us(10);
    PORTF |= (1<<TRIG_ULTR); // TRIG = HIGH
    _delay_us(10);
    PORTF &= ~(1<<TRIG_ULTR);
    while(!(PINF & (1<<ECHO_ULTR))); // ECHO = HIGH면
    TCNT1 = 0x0000; // Counter / Timer 1 값 0으로 초기화
    while(PINF & (1<<ECHO_ULTR)); // ECHO = LOW면
    TCCR1B = 0x00; // Counter / Timer 1 클록 정지
    return distance = (int)(TCNT1 / 2 / 5.8); // 거리값보정
}
/*
모터: motor_in1, motor_in2 port
1. 거리 20cm 이내에 사물이 존재 & 온도 13.0도 이하에서 자동문 동작
2. 자동문 동작 안함 (온도가 높다면 거리내에 사물이 있어도 작동하지 않음)
*/
void move_motor(int distance, int temperature)
{
    TCNT2 = 0;
    if(distance>=0 && distance<=DISTANCE && temperature <= TEMPERATURE)
    PORTB = MOTOR_MOVE;
    else
    PORTB = MOTOR_STOP;
}
int get_temp() // ic2 통신을 통해 읽은 데이터를 온도로 return 한다
{
    int byte_data;
    byte_data = read_twi(ATS75_TEMP_REG);
    return display_fnd_temp(byte_data);
}
// FND에 온도 표시 & 온도를 (온도:섭씨 * 10) 인 int 값으로 return
int display_fnd_temp(int value)
{
    char digit[12] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7c, 0x07, 0x7f, 0x67, 0x40,
    0x00};
    char fnd_sel[4] = {0x01, 0x02, 0x04, 0x08};
    char value_int, value_deci, num[4];
    int i;
    if((value & 0x8000) != 0x8000) num[3] = 11;
    else{
        num[3] = 10;
        value = (~value)-1;
    }
    value_int = (char)((value & 0x7f00) >> 8);
    value_deci = (char)(value & 0x00ff);
    num[2] = (value_int / 10) % 10;
    num[1] = value_int % 10;
    num[0] = ((value_deci & 0x80) == 0x80) * 5;
    for(i=0; i<4; i++)
    {
        PORTC = digit[num[i]];
        PORTG = fnd_sel[i];
        if(i==1) PORTC |= 0x80;
        _delay_ms(2);
    }
    return num[2] * 100 + num[1] * 10 + num[0] * 1;
}
