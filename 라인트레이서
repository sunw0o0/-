#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

// 1. 매크로 및 상수 정의
#define BASE_SPEED        160   // 직진 기본 속도
#define TURN_SPEED        200   // 회전할 때 빠른 쪽 바퀴 속도
#define SLOW_SPEED        -40   // 회전할 때 느린 쪽 바퀴 속도
#define MAX_SPEED         220   // 최대 제한 속도
#define TURN_THRESHOLD    300   // 회전 판단 기준
#define ALL_DETECT_THRESHOLD 500 // 교차로 센서 감지 판단 기준
#define CROSS_TURN_SPEED  200   // 1~3번째 교차로 회전 속도
#define CROSS_TURN_SLOW   -40   // 1~3번째 교차로 느린 쪽 속도
#define IGNORE_FORWARD_TIME 800 // 4번째 감지 후 전진 시간(ms)
#define IGNORE_FORWARD_SPEED 100
#define AVOID_TURN_SPEED  190   // 차선구간 회피 시 빠른 쪽 속도
#define AVOID_SLOW_SPEED  -80   // 차선구간 회피 시 느린 쪽 속도
#define LEFT_HIT_THRESHOLD 350  // 왼쪽 센서 감지 기준
#define ESCAPE_TURN_TIME  300
#define ESCAPE_FORWARD_SPEED 90

//PSD 장애물 감지 기준값 (높일수록 더 바짝 다가가서 정지합니다)
#define PSD_THRESHOLD     400

#define s_move 0 // 직진
#define b_move 1 // 후진
#define l_move 2 // 좌회전
#define r_move 3 // 우회전

#define SENSOR_COUNT 6
#define FILTER_SIZE  6
#define BLACK_THRESHOLD 300 // 검은 선 감지 판단 기준

#define CAL_SPEED     100
#define CAL_STEP_TIME 300
#define CAL_REPEAT    3

// 2. 전역 변수
int sensor_to_led[SENSOR_COUNT] = {1, 2, 3, 4, 5, 6};
int sensor_min[SENSOR_COUNT] = {1023, 1023, 1023, 1023, 1023, 1023};
int sensor_max[SENSOR_COUNT] = {0, 0, 0, 0, 0, 0};
int recent_values[SENSOR_COUNT][FILTER_SIZE] = {0};
int save_index = 0;
int left_hit_count = 0;
int left_hit_cooldown = 0;
int right_hit_count = 0;
int right_hit_cooldown = 0;
uint8_t in_lane_mode = 0;   // 1이면 차선구간(선 피하기) 모드
uint8_t crossing_count = 0; // 교차로 통과 횟수 카운터

// 반전 트랙 플래그 (0: 검은선/흰배경, 1: 흰선/검은배경)
uint8_t is_inverted_track = 0;

// 와리가리 스캔용 타이머 전역 변수
int scan_timer = 0;

const int sensor_weights[SENSOR_COUNT] = {-2500, -1500, -500, 500, 1500, 2500};
int last_valid_position = 0;

// 3. 함수 선언
void Motor_Init(void);
void moter(int mode, int speed);
void moter_raw(int right_speed, int left_speed);
void LED_Init(void);
void LED_Update(void);
void calibration(void);
int ADC_Read(int channel);
int Apply_Filter(int sensor_num, int new_value);
int Get_Line_Position(void);
int Check_All_Detected(void);
int Get_Sensor_Line_Level(int i, int filtered_val);
void Obstacle_Init(void);
uint8_t Obstacle_Detected(void);

// 4. main 함수
int main(void)
{
	DDRF &= ~0x7E; // PF1~PF6 입력 설정
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	Motor_Init();
	LED_Init();
	Obstacle_Init();
	
	_delay_ms(100);

	calibration();
	
	// ---- 2. 주 메인 루프 ----
	while (1)
	{
		// 장애물 감지 루틴
		if (Obstacle_Detected())
		{
			PORTA &= ~(1 << 7); // 감지 알림용 LED ON
			moter_raw(0, 0);
			_delay_ms(10);

			while (Obstacle_Detected())
			{
				moter_raw(0, 0);
				_delay_ms(10);
			}

			PORTA |= (1 << 7); // 장애물 제거 시 LED OFF

			// 2) 장애물이 치워지면 맨 왼쪽 센서(0번)에 선이 감지될 때까지 라인트레이싱
			while (1)
			{
				int pos = Get_Line_Position();
				if (pos > TURN_THRESHOLD)       moter_raw(SLOW_SPEED, TURN_SPEED);
				else if (pos < -TURN_THRESHOLD) moter_raw(TURN_SPEED, SLOW_SPEED);
				else                            moter_raw(BASE_SPEED, BASE_SPEED);

				int raw0 = ADC_Read(1);
				int line0 = Get_Sensor_Line_Level(0, raw0);

				if (line0 > LEFT_HIT_THRESHOLD)
				{
					moter_raw(0, 0); // 즉시 정지
					_delay_ms(50);
					break;
				}
				_delay_ms(2);
			}

			// 3) 회전 길 중앙까지 조금 더 직진
			moter_raw(BASE_SPEED, BASE_SPEED);
			_delay_ms(2000); // 중앙까지 이동하는 시간 - 실제로 테스트하며 조절

			moter_raw(0, 0);
			_delay_ms(50);

			// 4) 좌회전 수행
			moter_raw(SLOW_SPEED, TURN_SPEED);
			_delay_ms(400);

			moter_raw(0, 0);
			_delay_ms(50);

			// 4) 라인트레이싱을 하다가 5개 이상 IR 센서가 검은 선을 잡을 때까지 주행
			while (1)
			{
				int pos = Get_Line_Position();
				if (pos > TURN_THRESHOLD)       moter_raw(SLOW_SPEED, TURN_SPEED);
				else if (pos < -TURN_THRESHOLD) moter_raw(TURN_SPEED, SLOW_SPEED);
				else                            moter_raw(BASE_SPEED, BASE_SPEED);

				int count = 0;
				for (int i = 0; i < SENSOR_COUNT; i++)
				{
					int raw = ADC_Read(i + 1);
					int filtered = Apply_Filter(i, raw);
					if (Get_Sensor_Line_Level(i, filtered) > ALL_DETECT_THRESHOLD) count++;
				}
				save_index = (save_index + 1) % FILTER_SIZE;

				if (count >= 5)
				{
					moter_raw(0, 0);
					_delay_ms(50);
					break;
				}
				_delay_ms(2);
			}

			// 5) 180도 U턴 회전
			moter_raw(TURN_SPEED, -TURN_SPEED);
			_delay_ms(800);

			moter_raw(0, 0);
			_delay_ms(50);

			// 6) U턴 후 라인트레이싱 복귀
			while (1)
			{
				int pos = Get_Line_Position();
				if (pos > TURN_THRESHOLD)       moter_raw(SLOW_SPEED, TURN_SPEED);
				else if (pos < -TURN_THRESHOLD) moter_raw(TURN_SPEED, SLOW_SPEED);
				else                            moter_raw(BASE_SPEED, BASE_SPEED);

				int count = 0;
				for (int i = 0; i < SENSOR_COUNT; i++)
				{
					int raw = ADC_Read(i + 1);
					int filtered = Apply_Filter(i, raw);
					if (Get_Sensor_Line_Level(i, filtered) > ALL_DETECT_THRESHOLD) count++;
				}
				save_index = (save_index + 1) % FILTER_SIZE;

				if (count >= 5)
				{
					moter_raw(0, 0);
					_delay_ms(30);
					break;
				}
				_delay_ms(2);
			}

			// 7) 5개 센서 감지 시 좌회전 수행
			moter_raw(SLOW_SPEED, TURN_SPEED);
			_delay_ms(400);

			moter_raw(0, 0);
			_delay_ms(30);

			is_inverted_track = 1;

			continue;
		}
		
		int position = Get_Line_Position();

		// ---- 차선구간(선 피하기) 모드 ----
		if (in_lane_mode == 1)
		{
			int raw0 = ADC_Read(1);
			int filtered0 = Apply_Filter(0, raw0);
			int black0 = Get_Sensor_Line_Level(0, filtered0);

			int raw1 = ADC_Read(2);
			int filtered1 = Apply_Filter(1, raw1);
			int black1 = Get_Sensor_Line_Level(1, filtered1);

			int raw4 = ADC_Read(5);
			int filtered4 = Apply_Filter(4, raw4);
			int black4 = Get_Sensor_Line_Level(4, filtered4);

			int raw5 = ADC_Read(6);
			int filtered5 = Apply_Filter(5, raw5);
			int black5 = Get_Sensor_Line_Level(5, filtered5);

			int raw2 = ADC_Read(3);
			int filtered2 = Apply_Filter(2, raw2);
			int black2 = Get_Sensor_Line_Level(2, filtered2);

			int raw3 = ADC_Read(4);
			int filtered3 = Apply_Filter(3, raw3);
			int black3 = Get_Sensor_Line_Level(3, filtered3);

			int left_hit   = (black0 > LEFT_HIT_THRESHOLD || black1 > LEFT_HIT_THRESHOLD);
			int right_hit  = (black4 > LEFT_HIT_THRESHOLD || black5 > LEFT_HIT_THRESHOLD);

			// 오른쪽 센서 감지 카운터
			if (right_hit && right_hit_cooldown == 0)
			{
				right_hit_count++;
				right_hit_cooldown = 200;
			}
			if (right_hit_cooldown > 0) right_hit_cooldown--;

			// 오른쪽 센서 2번째 감지 시: 차선 탈출
			if (right_hit_count >= 2)
			{
				moter_raw(0, 0);
				_delay_ms(30);
				
				moter_raw(-100, -100);
				_delay_ms(180);

				moter_raw(160, 0);
				_delay_ms(1200);

				while (1)
				{
					moter_raw(160, 0);

					int raw2_tmp = ADC_Read(3);
					int black2_tmp = Get_Sensor_Line_Level(2, raw2_tmp);

					int raw3_tmp = ADC_Read(4);
					int black3_tmp = Get_Sensor_Line_Level(3, raw3_tmp);

					if (black2_tmp > BLACK_THRESHOLD || black3_tmp > BLACK_THRESHOLD)
					{
						moter_raw(0, 0);
						_delay_ms(30);
						break;
					}

					_delay_ms(2);
				}

				in_lane_mode = 0;
				left_hit_count = 0;
				right_hit_count = 0;
				right_hit_cooldown = 0;

				_delay_ms(2);
				continue;
			}

			// 평소 회피 동작
			if (left_hit)
			{
				moter_raw(-100, -100);
				_delay_ms(150);

				moter_raw(AVOID_SLOW_SPEED, AVOID_TURN_SPEED);
				_delay_ms(350);
				continue;
			}
			else if (right_hit)
			{
				moter_raw(-100, -100);
				_delay_ms(150);

				moter_raw(AVOID_TURN_SPEED, AVOID_SLOW_SPEED);
				_delay_ms(350);
				continue;
			}
			else
			{
				moter_raw(BASE_SPEED, BASE_SPEED);
			}

			_delay_ms(2);
			continue;
		}

		// ---- 교차로 판정 ----
		if (Check_All_Detected())
		{
			crossing_count++;

			if (crossing_count <= 3)
			{
				moter_raw(CROSS_TURN_SLOW, CROSS_TURN_SPEED);
				_delay_ms(400);

				moter_raw(BASE_SPEED, BASE_SPEED);
				_delay_ms(150);
			}
			else if (crossing_count == 4)
			{
				moter_raw(IGNORE_FORWARD_SPEED, IGNORE_FORWARD_SPEED);
				_delay_ms(IGNORE_FORWARD_TIME);

				in_lane_mode = 1;
			}

			continue;
		}

		// ---- 평소 라인트레이싱 ----
		if (position == 9999)
		{
			// 반전 트랙 와리가리 스캔
			scan_timer++;
			if ((scan_timer / 200) % 2 == 0)
			{
				moter_raw(140, -140);
			}
			else
			{
				moter_raw(-140, 140);
			}
		}
		else
		{
			scan_timer = 0;

			if (position > TURN_THRESHOLD)
			{
				moter_raw(SLOW_SPEED, TURN_SPEED);
			}
			else if (position < -TURN_THRESHOLD)
			{
				moter_raw(TURN_SPEED, SLOW_SPEED);
			}
			else
			{
				moter_raw(BASE_SPEED, BASE_SPEED);
			}
		}

		_delay_ms(2);
	}

	return 0;
}

// 5. 함수 정의
int Get_Sensor_Line_Level(int i, int filtered_val)
{
	int range = sensor_max[i] - sensor_min[i];
	if (range < 1) range = 1;

	int line_level = 0;
	if (is_inverted_track == 0)
	{
		line_level = (sensor_max[i] - filtered_val) * 1000 / range;
	}
	else
	{
		line_level = (filtered_val - sensor_min[i]) * 1000 / range;
	}

	if (line_level < 0)    line_level = 0;
	if (line_level > 1000) line_level = 1000;

	return line_level;
}

void Motor_Init(void)
{
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB5) | (1 << PB6);
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
}

void moter(int mode, int speed)
{
	if (speed > 200) speed = 200;
	if (speed < 0)   speed = 0;

	switch (mode)
	{
		case s_move:
		PORTB = (PORTB & ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3))) | (1 << PB0) | (1 << PB2);
		OCR1A = (uint8_t)speed;
		OCR1B = (uint8_t)speed;
		break;

		case b_move:
		PORTB = (PORTB & ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3))) | (1 << PB1) | (1 << PB3);
		OCR1A = (uint8_t)speed;
		OCR1B = (uint8_t)speed;
		break;

		case l_move:
		PORTB = (PORTB & ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3))) | (1 << PB0) | (1 << PB3);
		OCR1A = (uint8_t)speed;
		OCR1B = (uint8_t)speed;
		break;

		case r_move:
		PORTB = (PORTB & ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3))) | (1 << PB1) | (1 << PB2);
		OCR1A = (uint8_t)speed;
		OCR1B = (uint8_t)speed;
		break;

		default:
		PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3));
		OCR1A = 0;
		OCR1B = 0;
		break;
	}
}

void moter_raw(int right_speed, int left_speed)
{
	if (left_speed > 0)
	{
		left_speed = (left_speed * 90) / 100;
		if (left_speed < 40) left_speed = 40;
	}
	else if (left_speed < 0)
	{
		left_speed = (left_speed * 90) / 100;
		if (left_speed > -40) left_speed = -40;
	}

	if (right_speed > MAX_SPEED)  right_speed = MAX_SPEED;
	if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

	if (left_speed > MAX_SPEED)  left_speed = MAX_SPEED;
	if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;

	if (right_speed >= 0) {
		PORTB = (PORTB & ~((1 << PB0) | (1 << PB1))) | (1 << PB0);
		} else {
		PORTB = (PORTB & ~((1 << PB0) | (1 << PB1))) | (1 << PB1);
		right_speed = -right_speed;
	}

	if (left_speed >= 0) {
		PORTB = (PORTB & ~((1 << PB2) | (1 << PB3))) | (1 << PB2);
		} else {
		PORTB = (PORTB & ~((1 << PB2) | (1 << PB3))) | (1 << PB3);
		left_speed = -left_speed;
	}

	OCR1A = (uint8_t)right_speed;
	OCR1B = (uint8_t)left_speed;
}

void LED_Init(void)
{
	DDRA  = 0xFF;
	PORTA = 0xFF;
}

void LED_Update(void)
{
	for (int i = 0; i < SENSOR_COUNT; i++)
	{
		int raw      = ADC_Read(i + 1);
		int filtered = Apply_Filter(i, raw);

		if (filtered < sensor_min[i]) sensor_min[i] = filtered;
		if (filtered > sensor_max[i]) sensor_max[i] = filtered;

		int line_level = Get_Sensor_Line_Level(i, filtered);

		int led_pin = sensor_to_led[i];
		if (line_level > BLACK_THRESHOLD)
		{
			PORTA &= ~(1 << led_pin);
		}
		else
		{
			PORTA |= (1 << led_pin);
		}
	}
	save_index = (save_index + 1) % FILTER_SIZE;
}

void calibration(void)
{
	moter(l_move, CAL_SPEED);
	for (int t = 0; t < CAL_STEP_TIME; t += 10)
	{
		LED_Update();
		_delay_ms(10);
	}

	for (int rep = 0; rep < CAL_REPEAT; rep++)
	{
		moter(r_move, CAL_SPEED);
		for (int t = 0; t < (CAL_STEP_TIME * 2); t += 10)
		{
			LED_Update();
			_delay_ms(10);
		}

		moter(l_move, CAL_SPEED);
		for (int t = 0; t < (CAL_STEP_TIME * 2); t += 10)
		{
			LED_Update();
			_delay_ms(10);
		}
	}

	moter(r_move, CAL_SPEED);
	for (int t = 0; t < CAL_STEP_TIME; t += 10)
	{
		LED_Update();
		_delay_ms(10);
	}

	moter(s_move, 0);
	_delay_ms(300);
}

//ADC 읽기 안정화 정밀 보정
int ADC_Read(int channel)
{
	// ADMUX 초기화 및 AVCC 레퍼런스 세팅 (0~7 채널 선택)
	ADMUX = (1 << REFS0) | (channel & 0x07);
	_delay_us(100);

	// 변환 시작
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));

	// 채널 변경 오차 방지를 위해 한번 더 샘플링
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));

	return ADC;
}

int Apply_Filter(int sensor_num, int new_value)
{
	recent_values[sensor_num][save_index] = new_value;

	int total = 0;
	for (int i = 0; i < FILTER_SIZE; i++)
	{
		total += recent_values[sensor_num][i];
	}

	return total / FILTER_SIZE;
}

int Get_Line_Position(void)
{
	int32_t weighted_sum = 0;
	int32_t total_sum = 0;
	int line_detected_count = 0;

	int line_levels[SENSOR_COUNT];
	for (int i = 0; i < SENSOR_COUNT; i++)
	{
		int raw      = ADC_Read(i + 1);
		int filtered = Apply_Filter(i, raw);
		line_levels[i] = Get_Sensor_Line_Level(i, filtered);

		int led_pin = sensor_to_led[i];
		if (line_levels[i] > BLACK_THRESHOLD)
		{
			PORTA &= ~(1 << led_pin);
			line_detected_count++;
		}
		else
		{
			PORTA |= (1 << led_pin);
		}

		if (line_levels[i] > 200)
		{
			weighted_sum += (int32_t)line_levels[i] * sensor_weights[i];
			total_sum += line_levels[i];
		}
	}
	save_index = (save_index + 1) % FILTER_SIZE;

	// 반전 트랙 전용 직진 & 스캔
	if (is_inverted_track == 1)
	{
		if (line_levels[2] > BLACK_THRESHOLD || line_levels[3] > BLACK_THRESHOLD)
		{
			last_valid_position = 0;
			return 0;
		}
		return 9999;
	}

	// 일반 트랙 조향
	if (total_sum == 0 || line_detected_count == 0)
	{
		return (last_valid_position < 0) ? -2500 : 2500;
	}

	int position = (int)(weighted_sum / total_sum);
	last_valid_position = position;

	return position;
}

int Check_All_Detected(void)
{
	int count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++)
	{
		int raw = ADC_Read(i + 1);
		int filtered = Apply_Filter(i, raw);

		int line_level = Get_Sensor_Line_Level(i, filtered);

		if (line_level > ALL_DETECT_THRESHOLD)
		{
			count++;
		}
	}
	save_index = (save_index + 1) % FILTER_SIZE;

	return (count >= 5) ? 1 : 0;
}

// 1. PSD 센서 초기화 (PF0 핀을 ADC 아날로그 입력으로 설정)
void Obstacle_Init(void)
{
	DDRF &= ~(1 << PF0);  // PF0 입력 설정
	PORTF &= ~(1 << PF0); // 내부 풀업 저항 OFF
}

uint8_t Obstacle_Detected(void)
{
	int psd_val = ADC_Read(0); // PF0(ADC0 채널) 값 읽기

	if (psd_val > PSD_THRESHOLD)
	{
		return 1; // 장애물 있음
	}
	return 0; // 장애물 없음
}
