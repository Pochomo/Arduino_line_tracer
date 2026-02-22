// 카메라 픽셀 수
#define N_PIX 128

// Camera (PORT A)
// Digital 22
#define CLK PA0
// Digital 23  
#define SI PA1

// Stepper Motors (PORT D)
// step dir를 좌 우 모터 각각 한 쌍씩
#define STEP_L PD0
#define DIR_L PD1
#define STEP_R PD2
#define DIR_R PD3

// Enable (PORT H)
#define EN PH0

// DDA Constants
// 모터 최대 주파수 제한
#define F_MAX 18000.0
// 타이머 ISR 커리어 주파수 : 40kHz
#define F_CARRIER 40000.0
// DDA의 파라미터 B
#define DDA_B 1024

// 기본 주행 속도로 목표 step 주파수
const float BASE_F = 12000.0;

-[]
// 가속도(10ms 루프마다 current_speed_var를 증가)
const float ACCEL_RATE = 900.0;

// PD 제어 게인
// u = Kp*e + Kd*de
// 비례 게인 (에러에 비례해서 조향)
const float KP_TURN = 140.0;
// 미분 게인 (에러 변화율에 반응, 진동 억제)
const float KD_TURN = 64.0;

// 감속 로직 파라미터: 코너에서 감속하기 위해 추가
// 에러에 따른 감속 민감도
// speed_scale = 1 - K_SPEED*|e| 사용
const float K_SPEED = 0.01;
// 코너에서 최소 속도 비율 아무리 심한 커브도 이 이하로 낮추지 않는다.
const float MIN_SCALE = 0.20;

// 라인을 완전히 놓쳤을 때 쓸 에러값 혹시 모를 상황을 대비해 추가
const float FAILSAFE_ERROR = 100.0;

// System Timing 제어 주기 10ms
const float ts = 0.01;
uint32_t MicrosSampleTime;
uint32_t time_out;

// 주행 상태 플래그
volatile bool g_is_running = false;

// ######################
// 라인 스캔 카메라 클래스
// ######################
class LineScanCamera {
public:
  // 카메라에서 읽은 128개 픽셀 데이터
  uint16_t camera_data[N_PIX];
  // 감지된 흰색 구간의 최대 길이
  int max_continuous_white;
  // 계산된 라인의 중심 위치 픽셀 인덱스를 기준으로
  float line_center;
  // 화면 중심으로부터의 오차
  float line_error;
  // 라인을 놓쳤을 때 복귀 방향 기억용
  int last_known_direction;
  // 카메라 노출 시간 (자동 조절됨)
  uint16_t exposure_time_us;
  // 현재 프레임의 최대 밝기값
  uint16_t max_val;

  LineScanCamera() {
    // 초기 중심은 63.5
    line_center = (N_PIX - 1) / 2.0;
    line_error = 0.0;

    // 초기 노출 시간은 환경에 따라 runAutoExposure()가 계산
    exposure_time_us = 1000;
    // 정지선 판별용 변수
    max_continuous_white = 0;
    last_known_direction = 0;
  }

  // 라인 스캔 카메라 하드웨어 초기화
  void init() {
    // PORTA CLK, SI 핀을 출력으로 설정
    DDRA |= _BV(CLK) | _BV(SI);
    adcInit();
  }


  // 카메라에서 한 프레임 읽고 라인 위치 계산
  void update() {
    // SI, CLK 신호 생성 - 새로운 프레임 시작 신호
    PORTA |= _BV(SI);
    PORTA |= _BV(CLK);
    PORTA &= ~_BV(SI);
    // 노출 시간 대기
    delayMicroseconds(exposure_time_us);

    // 0 ~ 127 픽셀 데이터 순차적으로 읽기
    for (int i = 0; i < N_PIX + 1; i++) {
      // ADC 변환 시작
      if (i < N_PIX) {
        ADCSRA |= _BV(ADSC);
        while (ADCSRA & _BV(ADSC));
        // 픽셀 데이터 저장
        camera_data[i] = ADCW;
      } else {
        ADCSRA |= _BV(ADSC);
        while (ADCSRA & _BV(ADSC));
        // 더미 변환 결과로 저장하지 않는다
        int temp = ADCW;
      }
      // 다음 픽셀로 이동
      PORTA &= ~_BV(CLK);
      PORTA |= _BV(CLK);
    }
    PORTA &= ~_BV(CLK);

    // 무게중심법으로 라인의 중심 계산
    float c;
    bool found = computeLineCenter(&c);

    if (found) {
      // 라인을 찾았으면 중심과 에러 업데이트
      line_center = c;
      // 화면 중앙(63.5) 기준 오차 왼쪾이면 음수 오른쪽이면 양수를 뜻한다
      line_error = line_center - (float)(N_PIX - 1) / 2.0;

      // 라인 놓쳤을 경우 복귀용으로 마지막 방향 기억
      if (line_error < 0) last_known_direction = -1;
      else last_known_direction = 1;
    } else {
      // 라인을 못찾으면 위에서 초기화한 FAILSAFE_ERROR값을 넣어 복귀를 유도하고자했다.
      if (last_known_direction == -1) line_error = -FAILSAFE_ERROR;
      else if (last_known_direction == 1) line_error = FAILSAFE_ERROR;
    }
    // 조명 변화에 맞춰 노출 자동 조절
    runAutoExposure();
  }

  float getLineError() const {
    // 제어에서 라인오차만 사용
    return line_error;
  }

private:

  // ADC 레지스터 설정
  void adcInit() {
    // REFS0를 1로
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADIF) | (1 << ADPS2) | (1 << ADPS0);
  }

  // 픽셀 데이터에서 흰 라인의 중심 위치 계산
  bool computeLineCenter(float *center_out) {
    // ROI를 활용해 사용할 픽셀 번위를 제한한다. 끝단에 있는 잡음을 피하기 위해
    // 위 로직을 사용했다.
    const int ROI_START = 2;
    const int ROI_END = 125;
    uint16_t minv = 1023;
    max_val = 0;

    // 2~125 즉 ROI 범위 내 픽셀 정보 불러와 최대/최소 계산
    for (int k = ROI_START; k <= ROI_END; k++) {
      if (camera_data[k] < minv) minv = camera_data[k];
      if (camera_data[k] > max_val) max_val = camera_data[k];
    }

    // 최대 - 최소 < 60일 경우 라인이랑 배경 구분이 어려운 것이므로 라인 미검출로 판단
    if ((max_val - minv) < 60) {
      max_continuous_white = 0;
      return false;
    }

    // 최대/최소를 이용하여 임계값(th) 초기화
    // min부터 max 범위의 45%지점을 선택했다.
    uint16_t th = minv + (uint16_t)((max_val - minv) * 0.45);

    // 가장 긴 흰색 구간(blob) 찾기
    // 최대 blob 길이
    int max_blob_len = 0;
    // 최대 blob 시작 인덱스
    int max_blob_start = -1;
    // 최대 blob 끝 인덱스
    int max_blob_end = -1;
    // 현재 진행 중인 blob 길이
    int cur_len = 0;
    // 현재 blob 시작 인덱스
    int cur_start = -1;

    for (int k = ROI_START; k <= ROI_END; k++) {
      // 픽셀이 흰색인 경우 시작점 기록
      if (camera_data[k] > th) {
        if (cur_len == 0) cur_start = k;
        // 길이 + 1
        cur_len++;
      }
      // 픽셀이 검은색인 경우
      else {
        // cur의 길이가 저장된 blob길이보다 긴 경우 현재 cur값을 blob에 저장
        if (cur_len > max_blob_len) {
          // 최대값 갱신
          max_blob_len = cur_len;
          max_blob_start = cur_start;
          max_blob_end = k - 1;
        }
        // 현재 길이 초기화
        cur_len = 0;
      }
    }

    // 만약 흰색선이 화면 맨 오른쪽 끝까지 이어지는 경우
    // else문을 거치지 못하고 for문이 종료되므로 마지막에 한번 체크
    if (cur_len > max_blob_len) {
      max_blob_len = cur_len;
      max_blob_start = cur_start;
      max_blob_end = ROI_END;
    }
    // 결과 저장
    max_continuous_white = max_blob_len;
    // 3픽셀 미만인 경우 노이즈 취급
    if (max_blob_len < 3) return false;

    float num = 0.0, den = 0.0;

    // 앞서 구한 blob 내에서만 계산
    for (int k = max_blob_start; k <= max_blob_end; k++) {
      // 가중치(w) = 픽셀 밝기 - 임계값 (더 밝은 픽셀 쪽으로 중심이 잡힘)
      float w = (float)(camera_data[k] - th);
      // 위치 x 가중치
      num += k * w;
      // 가중치 총합
      den += w;
    }
    // 분모가 0일 때 계산 불가
    if (den <= 0.0) return false;
    // 최종 센터 값 = (위치 X 밝기의 합)/ 밝기의 합
    *center_out = num / den;
    return true;
  }

  // 조명 환경이 바뀌어도 라인을 안정적으로 검출하기 위함
  void runAutoExposure() {
    // ADC (0~1023)을 기준으로 노출 시간 조절

    // 어두우면 (750 미만) 노출 시간 증가
    if (max_val < 750) exposure_time_us += 50;
    // 밝으면 (900 초과) 노출 시간 감소
    else if (max_val > 900 && exposure_time_us > 50) exposure_time_us -= 50;
    // 노출의 최대/최소 범위 제한
    if (exposure_time_us < 100) exposure_time_us = 100;
    if (exposure_time_us > 6000) exposure_time_us = 6000;
  }
};

// #######################
// class:STEPPER MOTOR DDA
// #######################
class DualStepperDDA {
public:
  // DDA 알고리즘 변수
  volatile uint16_t A_L, B_L, A_R, B_R;
  // 누적기
  volatile uint16_t accL, accR;

  DualStepperDDA() {
    // 초기값
    A_L = A_R = 0;
    // B는 고정값
    B_L = B_R = DDA_B;
    // 누적기 초기화
    accL = accR = 0;
  }

  void init() {
    // 모터 제어 핀들 출력으로 설정
    DDRD |= _BV(STEP_L) | _BV(DIR_L) | _BV(STEP_R) | _BV(DIR_R);
    DDRH |= _BV(EN);
    // 모터 드라이버 활성화
    PORTH &= ~_BV(EN);

    // 방향 설정 (좌우 반대 방향)
    PORTD |= _BV(DIR_L);
    PORTD &= ~_BV(DIR_R);

    // Timer1 CTC 모드로 설정
    timer1_init();
  }

  // 좌우 모터의 목표 주파수 설정
  void setDDA(float fL, float fR) {
    // 주파수 범위 제한
    if (fL < 0.0) fL = 0.0;
    else if (fL > F_MAX) fL = F_MAX;
    if (fR < 0.0) fR = 0.0;
    else if (fR > F_MAX) fR = F_MAX;

    // DDA A 계산 A = fd * B / fc
    // 0.5는 반올림으로 정수화 오차를 줄인다
    uint16_t AL = (uint16_t)(fL * (float)DDA_B / F_CARRIER + 0.5);
    uint16_t AR = (uint16_t)(fR * (float)DDA_B / F_CARRIER + 0.5);

    // 0이 되면 안 되는 경우 처리
    if (AL == 0 && fL > 0.1) AL = 1;
    if (AR == 0 && fR > 0.1) AR = 1;

    // Timer1 ISR이 실행 중 A_L, A_R이 바뀌면 값이 섞일 수 도 있으니깐
    // 인터럽트를 잠깐 cli(막고)하고 다시 갱신하고 sei(허용)
    cli();
    A_L = AL;
    A_R = AR;
    sei();
  }

private:
  void timer1_init() {
    cli();
    // Timer1 레지스터 초기화
    TCCR1A = 0;
    TCCR1B = 0;
    // 카운터 초기화
    TCNT1 = 0;
    // 16MHz / (8 x (49 + 1)) = 40kHz
    OCR1A = 49;
    // CTC mode, Prescaler 8
    TCCR1B |= (1 << WGM12) | (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
    // 인터럽트 허용
    sei();
  }
};

// 카메라 및 모터 전역객체 생성
LineScanCamera g_cam;
DualStepperDDA g_motor;

//DDA 알고리즘으로 스텝 펄스 생성
ISR(TIMER1_COMPA_vect) {
  // 왼쪽 모터 DDA
  g_motor.accL += g_motor.A_L;
  if (g_motor.accL >= g_motor.B_L) {
    g_motor.accL -= g_motor.B_L;
    // 스텝 펄스 생성
    PORTD |= _BV(STEP_L);
  } else {
    // 펄스 미발생 시 low
    PORTD &= ~_BV(STEP_L);
  }

  // 오른쪽 모터 DDA
  g_motor.accR += g_motor.A_R;
  if (g_motor.accR >= g_motor.B_R) {
    g_motor.accR -= g_motor.B_R;
    // 스텝 펄스 생성
    PORTD |= _BV(STEP_R);
  } else {
    PORTD &= ~_BV(STEP_R);
  }
}

// ==========================================
//   MAIN SETUP & LOOP
// ==========================================
void setup() {
  // 하드웨어 초기화 카메라 및 모터 클래스 활용해 init()으로 초기화 진행
  g_cam.init();
  g_motor.init();

  // 자동 노출 안정화를 위한 라인 스캔 센서 워밍업
  for (int i = 0; i < 20; i++) {
    g_cam.update();
    delay(10);
  }

  // 제어 주기 타이밍 초기화
  MicrosSampleTime = (uint32_t)(ts * 1e6);
  time_out = micros() + MicrosSampleTime;

  // 출발 딜레이
  delay(1);
}

// 현재 주행 바퀴 수
int lap_count = 0;
// 목표 바퀴 수
const int TARGET_LAPS = 2;

// 정지선 인식시 일정 시간동안 다시 정지선을 인식하지 않기 위한 타임 스탬프
unsigned long last_mark_time = 0;

// 현재 실제 주행 속도 (가속 제어용)
float current_speed_var = 0.0;

void loop() {
  // D제어를 위한 이전 에러값 기록 변수
  static float prev_error = 0.0;

  // 센서 업데이트
  g_cam.update();

  // 흰 구간이 너무 짧으면 정지선으로 인식
  bool is_stop_line = (g_cam.max_continuous_white < 18);

  // 출발 로직
  if (!g_is_running) {
    g_is_running = true;
    last_mark_time = millis();
    lap_count = 0;
    prev_error = 0.0;
    // 속도 0부터 시작
    current_speed_var = 0.0;
  }

  // 주행 중 제어
  if (g_is_running) {
    // 정지선 인식과 랩 카운트 (타임스탬프로 중복인식 방지)
    if (is_stop_line && (millis() - last_mark_time > 2000)) {
      lap_count++;
      last_mark_time = millis();

      // 목표 바퀴 달성시 정지
      if (lap_count >= TARGET_LAPS) {
        g_motor.setDDA(0.0, 0.0);
        g_is_running = false;
        while (1);
      }
    }

    // PD 제어
    // 현재 라인 오차
    float e = g_cam.getLineError();
    // 에러 변화율
    float de = e - prev_error;
    prev_error = e;

    // u 계산 좌 우 다른 속도를 주기 위한 제어 입력 계산
    float u = KP_TURN * e + KD_TURN * de;

    /// 감속 로직 - 에러가 클수록 (급커브) 속도 낮춤
    float abs_e = abs(e);
    float speed_scale = 1.0 - K_SPEED * abs_e;
    if (speed_scale < MIN_SCALE) speed_scale = MIN_SCALE;
    if (speed_scale > 1.0) speed_scale = 1.0;

    // 감속 적용된 base 주파수
    float target_base = BASE_F * speed_scale;


    // 천천히 가속해서 출발 시 바퀴 헛도는 거 방지
    if (current_speed_var < target_base) {
      // 서서히 가속
      current_speed_var += ACCEL_RATE;
      if (current_speed_var > target_base) current_speed_var = target_base;
    } else {
      // 감속은 즉시 반영 (코너 진입 시 밀림 방지)
      current_speed_var = target_base;
    }

    // 최종 속도 적용
    g_motor.setDDA(current_speed_var + u, current_speed_var - u);
  }

  // 10ms 주기 유지
  while (micros() < time_out);
  time_out = micros() + MicrosSampleTime;
}