//���õ�C��ͷ�ļ�
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//Log��Ҫ���õ�ͷ�ļ�
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//APP��ʱ����Ҫ���õ�ͷ�ļ�
#include "app_timer.h"
#include "bsp_btn_ble.h"
//�㲥��Ҫ���õ�ͷ�ļ�
#include "ble_advdata.h"
#include "ble_advertising.h"
//��Դ������Ҫ���õ�ͷ�ļ�
#include "nrf_pwr_mgmt.h"
//SoftDevice handler configuration��Ҫ���õ�ͷ�ļ�
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
//����д��ģ����Ҫ���õ�ͷ�ļ�
#include "nrf_ble_qwr.h"
//GATT��Ҫ���õ�ͷ�ļ�
#include "nrf_ble_gatt.h"
//���Ӳ���Э����Ҫ���õ�ͷ�ļ�
#include "ble_conn_params.h"
//����͸����Ҫ���õ�ͷ�ļ�
#include "my_ble_uarts.h"
//����FDSͷ�ļ�
#include "fds.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "app_uart.h"
#include "device_name_op.h"
#include "nrf_delay.h"
#include "nrfx_wdt.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_saadc.h"

#define AD_SCK          NRF_GPIO_PIN_MAP(0,5)
#define AD_DOUT         NRF_GPIO_PIN_MAP(0,6)
#define SENSOR_PW       NRF_GPIO_PIN_MAP(0,12)
#define CHG_IDLE        NRF_GPIO_PIN_MAP(0,15)
#define CHG_ING         NRF_GPIO_PIN_MAP(0,16)
#define LED_GN          NRF_GPIO_PIN_MAP(0,18)
#define LED_RD          NRF_GPIO_PIN_MAP(0,20)
#define POW_BTN         NRF_GPIO_PIN_MAP(0,28)
#define POW_OUT1        NRF_GPIO_PIN_MAP(0,30)
#define POW_OUT2        NRF_GPIO_PIN_MAP(0,14)

uint8_t Reset_Count = 0;
uint8_t BattType = 0;
uint8_t Sys_State = 0;
uint8_t Charge_State = 0;
uint8_t Last_Charge_State = 0;
uint8_t Charge_Ing = 0;
uint8_t Charge_Flash = 0;
uint8_t Charge_Count = 0;
uint8_t Low_Power = 0;
uint8_t Batt_Count = 0;
uint8_t Lost_Count = 0;
uint8_t Ack_Flag = 0;
uint8_t Serial_Flag = 0;
uint8_t Cmd_Flag = 0;
uint8_t Set_Data = 0;
uint8_t Led_Flash = 0;
uint8_t NO_Data = 0;
int16_t Batt_Cap = 60;
int16_t Weight_Data = 0;
int16_t Adc_Value = 370;
int16_t Sleep_Count = 0;
int16_t Max_Adc_Value = 0;
uint16_t Charge_Over = 0;

#define SLEEP_VALUE 600 //600s=10min


extern void  HX71708_Sampling(unsigned long long  sti);


extern void Avg_Filter(void);
extern void Read_Data(void);
extern unsigned long Uart_Send(unsigned char);
extern void Save_Data(unsigned char);
extern void Clear_Data(void);
extern void Init_Data(void);
extern void Lock_Data(void);

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(16, UNIT_1_25_MS)    //��С���Ӽ��
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(32, UNIT_1_25_MS)    //������Ӽ��
#define SLAVE_LATENCY                   0                                  //�ӻ��ӳ� 
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(500, UNIT_10_MS)     //�ල��ʱ
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)              //�����״ε���sd_ble_gap_conn_param_update()�����������Ӳ����ӳ�ʱ�䣨5�룩
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)             //����ÿ�ε���sd_ble_gap_conn_param_update()�����������Ӳ����ļ��ʱ�䣨30�룩
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                  //����������Ӳ���Э��ǰ�������Ӳ���Э�̵���������3�Σ�

#define APP_BLE_OBSERVER_PRIO           3               //Ӧ�ó���BLE�¼����������ȼ�,Ӧ�ó������޸ĸ���ֵ

#define UART_TX_BUF_SIZE                256             //���ڷ��ͻ����С���ֽ�����
#define UART_RX_BUF_SIZE                256             //���ڽ��ջ����С���ֽ�����

//����stack dump�Ĵ������,��������ջ����ʱȷ����ջλ��
#define DEAD_BEEF                       0xDEADBEEF     
#define BLE_CONN_PW_LEVEL               4
               
BLE_UARTS_DEF(m_uarts, NRF_SDH_BLE_TOTAL_LINK_COUNT);    //��������Ϊm_uarts�Ĵ���͸������ʵ��
NRF_BLE_GATT_DEF(m_gatt);                                //��������Ϊm_gatt��GATTģ��ʵ��
NRF_BLE_QWR_DEF(m_qwr);                                  //����һ������Ϊm_qwr���Ŷ�д��ʵ��


APP_TIMER_DEF(m_app_timer_id);
#define  APP_TIMER_INTERVAL APP_TIMER_TICKS  (100) 
APP_TIMER_DEF(m_avg_timer_id);
#define  AVG_TIMER_INTERVAL APP_TIMER_TICKS  (33)  // 30Hz

nrfx_wdt_channel_id m_channel_id;
nrf_saadc_value_t  saadc_val;

//�ñ������ڱ������Ӿ��,��ʼֵ����Ϊ������
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
//���͵�������ݳ���
static uint16_t   m_ble_uarts_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;


extern unsigned long  chData[16];


void  deep_sleep(void) ;


void Set_Led(uint8_t state)
{
  if(state == 0)  //����״̬
  {
    if(Charge_Ing == 1)
    {
      nrf_gpio_pin_set(LED_RD);
      nrf_gpio_pin_set(LED_GN);
    }
    else
    {
      nrf_gpio_pin_clear(LED_GN);
      nrf_gpio_pin_clear(LED_RD);
    }
  }	
  if(state == 1)  //�㲥״̬
  {
    if(Charge_Ing == 1)
    {
      nrf_gpio_pin_set(LED_RD);
      nrf_gpio_pin_set(LED_GN);
    }
    else if(Low_Power == 0) 
    {
      nrf_gpio_pin_set(LED_GN);
      nrf_gpio_pin_clear(LED_RD);
    }
    else if(Low_Power == 1) 
    {
      nrf_gpio_pin_set(LED_RD);
      nrf_gpio_pin_clear(LED_GN);
    }
  }
  if(state == 2 || state == 3) //����״̬
  {
    if(Charge_Ing == 1)
    {
      if(Charge_Flash == 1)
      {
        nrf_gpio_pin_set(LED_RD);
        nrf_gpio_pin_set(LED_GN);
        Charge_Flash = 0;
      }			
      else if(Charge_Flash == 0)
      {
        nrf_gpio_pin_clear(LED_RD);
        nrf_gpio_pin_clear(LED_GN);
        Charge_Flash = 1;
      }
    }
    else if(Low_Power == 0) 
    {
      nrf_gpio_pin_toggle(LED_GN);
      nrf_gpio_pin_clear(LED_RD);
    }
    else if(Low_Power == 1) 
    {
      nrf_gpio_pin_toggle(LED_RD);
      nrf_gpio_pin_clear(LED_GN);
    }
  }
}

//GAP������ʼ��,�ú���������Ҫ��GAP����,�����豸����,�����������ѡ���Ӳ���
static void gap_params_init(void)
{
  ret_code_t  err_code;
  //�������Ӳ����ṹ�����
  ble_gap_conn_params_t   gap_conn_params;	
	
  //�����豸����:��FLash�в����豸���Ƽ�¼,����豸������Ч,ʹ�ø��豸��������GAP�豸����,����ʹ��Ĭ�ϵ��豸����
  device_name_set(); // 1-1 open mode
	
  //������ѡ���Ӳ���,����ǰ������gap_conn_params
  memset(&gap_conn_params, 0, sizeof(gap_conn_params));
  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;//��С���Ӽ��
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;//��С���Ӽ��
  gap_conn_params.slave_latency     = SLAVE_LATENCY;    //�ӻ��ӳ�
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT; //�ල��ʱ
  //����Э��ջAPI sd_ble_gap_ppcp_set����GAP����
  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}
//GATT�¼�������,�ú����д���MTU�����¼�
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
}

//��ʼ��GATT����ģ��
static void gatt_init(void)
{
  //��ʼ��GATT����ģ��
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  //��麯�����صĴ������
  APP_ERROR_CHECK(err_code);
  //����ATT MTU�Ĵ�С,�������õ�ֵΪ247
  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

//�Ŷ�д���¼�������,���ڴ����Ŷ�д��ģ��Ĵ���
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
  //���������
  NRF_LOG_INFO("App Reset 4");
  NVIC_SystemReset();
  APP_ERROR_HANDLER(nrf_error);
}

//�����¼��ص�����,���ڳ�ʼ��ʱע��,�ú������ж��¼����Ͳ����д���
//�����յ����ݳ��ȴﵽ�趨�����ֵ���߽��յ����з���,����Ϊһ�����ݽ������,֮�󽫽��յ����ݷ��͸�����
void uart_event_handle(app_uart_evt_t * p_event)
{
  static uint8_t data_array[BLE_UARTS_MAX_DATA_LEN];
  static uint8_t index = 0;
  //�ж��¼�����
  switch (p_event->evt_type)
  {
    case APP_UART_DATA_READY://���ڽ����¼�
         UNUSED_VARIABLE(app_uart_get(&data_array[index]));
         index++;
         //���մ������ݣ������յ����ݳ��ȴﵽm_ble_uarts_max_data_len���߽��յ����з�����Ϊһ�����ݽ������
         if((data_array[index - 1] == '4') || (index >= m_ble_uarts_max_data_len))
         {
           if(index > 1)
           {			
             if(data_array[0] == ':' && data_array[1] == '1' && data_array[2] == 'A' && data_array[3] == '1' && data_array[4] == '4') //��ȡ����
             {
               Serial_Flag = 1;
             }
             if(data_array[0] == ':' && data_array[1] == '1' && data_array[2] == 'A' && data_array[3] == '3' && data_array[4] == '4') //��ȡ�豸��Ϣ
             {
               Serial_Flag = 20;
             }
           }
           index = 0;
         }
         break;
    //ͨѶ�����¼������������
    case APP_UART_COMMUNICATION_ERROR:
         NRF_LOG_INFO("App Reset 1");
         NVIC_SystemReset();
         APP_ERROR_HANDLER(p_event->data.error_communication);
         break;
    //FIFO�����¼������������
    case APP_UART_FIFO_ERROR:
         NRF_LOG_INFO("App Reset 2");
         NVIC_SystemReset();
         APP_ERROR_HANDLER(p_event->data.error_code);
         break;
    default:
         break;
  }
}
//��������
void uart_config(void)
{
  uint32_t  err_code;
  //���崮��ͨѶ�������ýṹ�岢��ʼ��
  const app_uart_comm_params_t  comm_params =
  {
    RX_PIN_NUMBER,                 //����uart��������
    TX_PIN_NUMBER,                 //����uart��������
    RTS_PIN_NUMBER,                //����uart RTS����,���عرպ���Ȼ������RTS��CTS����,����������������,������������������,���������Կ���ΪIOʹ��
    CTS_PIN_NUMBER,                //����uart CTS����
    APP_UART_FLOW_CONTROL_DISABLED,//�ر�uartӲ������
    false,                         //��ֹ��ż����
    NRF_UART_BAUDRATE_115200       //uart����������Ϊ115200
  };
  //��ʼ�����ڣ�ע�ᴮ���¼��ص�����
  APP_UART_FIFO_INIT(&comm_params,UART_RX_BUF_SIZE,UART_TX_BUF_SIZE,uart_event_handle,APP_IRQ_PRIORITY_LOWEST,err_code);
  APP_ERROR_CHECK(err_code);
  
  app_uart_put('U');
  app_uart_put('A');
  app_uart_put('R');
  app_uart_put('T');
  app_uart_put(0x0D);
  app_uart_put(0x0A);
}

uint8_t ToHexStr(uint8_t num)
{
  if(num < 10)
  {
    return num + 48;
  }
  if(num >=10 && num < 16)
  {
    return num + 55;
  }
  return 48;
}

uint16_t time[3] = {0};
void Send_Edition()
{
  uint8_t Date[12] = __DATE__;
  time[0] = (Date[7] - '0')*1000 + (Date[8] - '0')*100 + (Date[9] - '0')*10 + (Date[10] - '0');
  if(Date[0] == 'J' && Date[1] == 'a' && Date[2] == 'n') time[1] = 1;
  if(Date[0] == 'F' && Date[1] == 'e' && Date[2] == 'b') time[1] = 2;
  if(Date[0] == 'M' && Date[1] == 'a' && Date[2] == 'r') time[1] = 3;
  if(Date[0] == 'A' && Date[1] == 'p' && Date[2] == 'r') time[1] = 4;
  if(Date[0] == 'M' && Date[1] == 'a' && Date[2] == 'y') time[1] = 5;
  if(Date[0] == 'J' && Date[1] == 'u' && Date[2] == 'n') time[1] = 6;
  if(Date[0] == 'J' && Date[1] == 'u' && Date[2] == 'l') time[1] = 7;
  if(Date[0] == 'A' && Date[1] == 'u' && Date[2] == 'g') time[1] = 8;
  if(Date[0] == 'S' && Date[1] == 'e' && Date[2] == 'p') time[1] = 9;
  if(Date[0] == 'O' && Date[1] == 'c' && Date[2] == 't') time[1] = 10; 
  if(Date[0] == 'N' && Date[1] == 'o' && Date[2] == 'v') time[1] = 11;
  if(Date[0] == 'D' && Date[1] == 'e' && Date[2] == 'c') time[1] = 12;
  if(Date[4] > '0')
  {
    time[2] = (Date[4] - '0')*10 + (Date[5] - '0');
  }
  else
  {
    time[2] = (Date[5] - '0');
  }
  NRF_LOG_INFO("%d-%d-%d",time[0],time[1],time[2]);
}


#define  BATT_M  (64)
unsigned short  Batt_adc[BATT_M] = {0};
unsigned long   Batt_sum = 0;
unsigned long   Batt_samp_n = 0;
unsigned short  Batt_val = 0;
float           Batt_f = 0;

void  battery_smooth(void)
{
	nrf_saadc_value_t    tmp_adcv = 0;
	nrfx_saadc_sample_convert(0, &tmp_adcv);
	
	if( ! Batt_samp_n) {
		memset(Batt_adc, 0, BATT_M*sizeof(unsigned short));
		Batt_sum = 0;
		Batt_val = 0;
	}
	
	Batt_sum -= Batt_adc[Batt_samp_n%BATT_M]; 
	Batt_adc[Batt_samp_n%BATT_M] = (unsigned short)tmp_adcv;
	Batt_sum += (unsigned short)tmp_adcv;	
	
	Batt_val = (Batt_samp_n > BATT_M) ? (Batt_sum>>6) : (Batt_sum / (Batt_samp_n +1) );
	
	++Batt_samp_n;	
}

static void m_avg_timeout_handler(void *p_context)
{
    static unsigned long long  nTim;
    
//  Avg_Filter();
	
    /*zj.huang 20230719*/
    HX71708_Sampling(nTim);
    /*zj.huang 20230711*/
    if( !(nTim%3))
        battery_smooth();
	
/*	
    long   TMP = 0;	
	char   tmpC[16] = {0};
	char   TXT[32] = {0};
	TMP = chData[0];	
	sprintf(tmpC, "%ld ", TMP);
	strcpy(TXT, tmpC);
			
//	TMP = Batt_val *0.713f;
//	sprintf(tmpC, "%ld ", TMP);
//	strcpy(TXT, tmpC);

	short  sn = strlen(TXT);
	for(short i = 0; i < sn; ++i)
	    app_uart_put(TXT[i]);
	app_uart_put(0x0D);
	app_uart_put(0x0A);
*/
    
	// hold on Sensor
//    nrf_gpio_pin_set( NRF_GPIO_PIN_MAP(0,12) );
	
    ++nTim;
}


static void m_app_timeout_handler(void *p_context)
{
  UNUSED_PARAMETER(p_context); //����
  uint8_t send_data[20]={0};
  uint8_t send_length = 0;
  uint32_t err_code;

  if(nrf_gpio_pin_read(POW_BTN) == 0)
  {
    if(Reset_Count == 100)
    {
      NRF_LOG_INFO("Btn Reset!");
    }
    if(Reset_Count++ > 110)
    {
      NVIC_SystemReset();
    }
  }
  else
  {
    Reset_Count = 0;
  }
  nrfx_wdt_channel_feed(m_channel_id);//ι��
  Read_Data();
  Weight_Data = Uart_Send(0);
  if(NO_Data == 1)
  {
    NRF_LOG_INFO("No Find Data!-%d ",Batt_Count);
    nrf_gpio_pin_set(SENSOR_PW);
    if(Batt_Count > 90)
    {
      NO_Data = 0;
      Init_Data();
      NRF_LOG_INFO("Init Data!");
    }
  }
  Batt_Count++;
  if(Batt_Count % 10 == 0)
  {
    if(Adc_Value <= 310 && Charge_State == 4)
    {
      Sleep_Count += 20;
    }
    else if(Sys_State <= 1 && Charge_State == 4)
    {
      Sleep_Count++;
    }
    else
    {
      Sleep_Count = 0;
    }
    if(Sleep_Count > SLEEP_VALUE)//����˯��
    {
      Sleep_Count = 0;
      Sys_State = 0;
      Set_Led(Sys_State);
      sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      ble_adv_stop();
	  /*zj.huang 20230711*/
	  deep_sleep();
      //nrf_gpio_pin_clear(POW_OUT1);
      //nrf_gpio_pin_clear(POW_OUT2);
    }
    Send_Edition();
    NRF_LOG_INFO("[Adc %d] [Cap %d] [CHG %d] [Sys %d] [Lost %d] [Weight %d] ",Adc_Value,Batt_Cap,Charge_State,Sys_State,Lost_Count,Weight_Data);
  }
  if(Batt_Count > 100)
  {
    Batt_Count = 0;
  }
  if(Batt_Count == 10)
  {
	/*zj.huang 20230711*/
	saadc_val = Batt_val;
	Adc_Value = Batt_val * 0.713f;

	
      if(Charge_Ing == 1)//����µĵ�������
      {
        Adc_Value -= 10;  //��׼����
      }
      if(Adc_Value < 351) Batt_Cap = 1;
	  else if(Adc_Value < 354) Batt_Cap = 5;
      else if(Adc_Value < 357) Batt_Cap = 10;
      else if(Adc_Value < 363) Batt_Cap = 20;
      else if(Adc_Value < 370) Batt_Cap = 40;
      else if(Adc_Value < 380) Batt_Cap = 60;
      else if(Adc_Value < 395) Batt_Cap = 80;
      else Batt_Cap = 100;
    
    if(Batt_Cap <= 10)		//�������ñ���λ
    {
      Low_Power = 1;
    }
    else
    {
      Low_Power = 0;
    }
	
  }
  //Batt_Cap = Adc_Value/10;if(Charge_State == 3) Batt_Cap += 10;//���Դ���

/*  
  if(nrf_gpio_pin_read(CHG_ING) == 0 && nrf_gpio_pin_read(CHG_IDLE) == 0)
  {
    Charge_State = 1;
  }
  if(nrf_gpio_pin_read(CHG_ING) == 0 && nrf_gpio_pin_read(CHG_IDLE) == 1)
  {
    Charge_State = 2;//���
  }
  if(nrf_gpio_pin_read(CHG_ING) == 1 && nrf_gpio_pin_read(CHG_IDLE) == 0)
  {
    Charge_State = 3;//����
  }
  if(nrf_gpio_pin_read(CHG_ING) == 1 && nrf_gpio_pin_read(CHG_IDLE) == 1)
  {
    Charge_State = 4;//�γ�
  }
*/
    /*zj.huang 20230809*/
    if( (nrf_gpio_pin_read(CHG_ING) == 1) && (nrf_gpio_pin_read(CHG_IDLE) == 0) ) {
        Charge_State = 2;//���
    }
    else if( (nrf_gpio_pin_read(CHG_ING) == 1) && (nrf_gpio_pin_read(CHG_IDLE) == 1) ) {
        Charge_State = 3;//����
    }
    else if( (nrf_gpio_pin_read(CHG_ING) == 0) && (nrf_gpio_pin_read(CHG_IDLE) == 1) ) {
        Charge_State = 4;//�γ�
    }
    else {
        Charge_State = 1;
    }
        
  if(Charge_State != Last_Charge_State)
  {
    if(Charge_State == 2)//����ʱ��ʾ��ɫ��
    {
      Charge_Ing = 1;
      Set_Led(Sys_State);
    }
    if(Charge_State == 3)//����ʱ
    {
    }
    if(Charge_State == 4)
    {
      if(Last_Charge_State == 2 || Last_Charge_State == 3)//�γ�ʱ
      {
        if(Sys_State <= 1)//�ǹ���״̬�ػ�
        {
          Charge_Ing = 0;
          Sys_State = 0;
          Set_Led(Sys_State);
          sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
          ble_adv_stop();
		  /*zj.huang 20230711*/
	      deep_sleep();
          //nrf_gpio_pin_clear(POW_OUT1);
          //nrf_gpio_pin_clear(POW_OUT2);
        }
      }
    }
    Last_Charge_State = Charge_State;
  }
  if(Sys_State <= 1)
  {
    Charge_Count++;
    if(Charge_Count >= 20)//�����Ͽ�ʱÿ2sˢ��һ��
    {
      Set_Led(Sys_State);
      Charge_Count = 0;
      if(Charge_State != 2)
      {
        Charge_Ing = 0;
        Set_Led(Sys_State);
      }
      if(Adc_Value >= 400 && Charge_State == 2)
      {
        if(Adc_Value <= Max_Adc_Value)
        {
          Charge_Over++;
        }
        else
        {
          Max_Adc_Value = Adc_Value;
          Charge_Over = 0;
        }
        if(Charge_Over > 400)//400*2s=800s=13min
        {
          Charge_Ing = 0;
          Set_Led(Sys_State);
          Charge_Over = 0;
          Max_Adc_Value = 0;
        }
      }
      else
      {
        Charge_Over = 0;
        Max_Adc_Value = 0;				
      }
    }
  }
  if(Sys_State == 2 || Sys_State == 3)
  {
    Led_Flash++;
    if(Led_Flash >= 5)
    {
      Set_Led(Sys_State);//��������ʱÿ0.5sˢ��һ��
      Led_Flash = 0;
    }
    if(Charge_State != 4)
    {
      Charge_Ing = 1;
    }
    else
    {
      Charge_Ing = 0;
    }
  }
  if(Cmd_Flag > 0 || (Ack_Flag > 0 && Ack_Flag < 18))
  {
    if(Ack_Flag == 5) Sys_State = 2;
    Lost_Count = 0;
    nrf_gpio_pin_set(SENSOR_PW);
  }
  if(Serial_Flag > 0 && Serial_Flag < 18)
  {
    if(Serial_Flag == 5) Sys_State = 3;
    Lost_Count = 0;
    nrf_gpio_pin_set(SENSOR_PW);
  }
  if(Lost_Count < 120)
  {
    Lost_Count++;
  }
  else
  {
    Lost_Count = 100;
    nrf_gpio_pin_clear(SENSOR_PW);
  }
  if(Lost_Count > 5)
  {
    if(Sys_State == 3)
    {
      Sys_State = 1;
      Serial_Flag = 0;
    }
  }
  if(Cmd_Flag == 1) //�궨
  {
    Save_Data(Set_Data);
    NRF_LOG_INFO("Set Data! -%d-",Set_Data);
    Cmd_Flag = 0;
  }
  if(Cmd_Flag == 2) //ȥƤ
  {
    Clear_Data();
    NRF_LOG_INFO("Clear Data!");
    Cmd_Flag = 0;
  }
  if((Ack_Flag > 0 && Ack_Flag < 18) || (Serial_Flag > 0 && Serial_Flag < 18)) //���ڽ��յ����ݷ��͸�BLE����//��������
  {
    send_length = 17;
    send_data[0] = ':';send_data[1] = '1';send_data[2] = 'A';send_data[3] = 'A';send_data[4] = '4';send_data[5] = '0';send_data[6] = '4';send_data[7] = '0';send_data[8] = '0';                       
    if(Weight_Data < 0) Weight_Data = 0;	
    send_data[9] = ToHexStr(Weight_Data/4096);
    send_data[10] = ToHexStr(Weight_Data%4096/256);
    send_data[11] = ToHexStr(Weight_Data%256/16);
    send_data[12] = ToHexStr(Weight_Data%16);
    send_data[13] = '0';//У��
    send_data[14] = '0';//У��
    send_data[15] = 0x0D;
    send_data[16] = 0x0A;
    if(Ack_Flag > 0) Ack_Flag++;
    if(Serial_Flag > 0) Serial_Flag++;
  }
  if(Ack_Flag == 20 || Serial_Flag == 20)
  {
    send_length = 19;
    send_data[0] = ':';send_data[1] = '1';send_data[2] = 'A';send_data[3] = 'A';send_data[4] = '4';send_data[5] = '0';send_data[6] = '4';send_data[7] = '0';send_data[8] = '0';
    send_data[9] = '6';send_data[10] = '5';send_data[11] = '6';send_data[12] = '5';
    send_data[13] = ToHexStr(Batt_Cap/16);
    send_data[14] = ToHexStr(Batt_Cap%16);
		
    send_data[15] = '1';
    send_data[16] = '5';
    send_data[17] = 0x0D;
    send_data[18] = 0x0A;
  }
  if(Ack_Flag == 21)
  {
    send_length = 18;
    send_data[0] = ':';send_data[1] = '1';send_data[2] = 'A';send_data[3] = '5';send_data[4] = '4';
    send_data[5] = ToHexStr(time[0]%100/10);send_data[6] = ToHexStr(time[0]%100%10);//��
    send_data[7] = ToHexStr(time[1]/10);send_data[8] = ToHexStr(time[1]%10);        //��
    send_data[9] = ToHexStr(time[2]/10);send_data[10] = ToHexStr(time[2]%10);       //��
    send_data[11] = ToHexStr(1);send_data[12] = ToHexStr(0);send_data[13] = ToHexStr(0);send_data[14] = ToHexStr(2);send_data[15] = ToHexStr(0);
    send_data[16] = 0x0D;
    send_data[17] = 0x0A;
  }
  if(Ack_Flag > 0)
  {
    if(Ack_Flag == 18 || Ack_Flag == 20 || Ack_Flag == 21)
    {
      Ack_Flag = 0;
    }
    do
    {						  
      uint16_t length = (uint16_t)send_length;
      err_code = ble_uarts_data_send(&m_uarts, send_data, &length, m_conn_handle);
      if((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
      {
        APP_ERROR_CHECK(err_code);
      }
    }
    while(err_code == NRF_ERROR_RESOURCES);
  }
  if(Serial_Flag > 0)
  {
    if(Serial_Flag == 18 || Serial_Flag == 20)
    {
      Serial_Flag = 0;
    }
    for(uint32_t i = 0; i < send_length; i++)
    {
      do
      {
        err_code = app_uart_put(send_data[i]);//��������		
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
        {
          APP_ERROR_CHECK(err_code);
        }
      }
      while (err_code == NRF_ERROR_BUSY);
    }
  }
}

//����͸���¼��ص�����,����͸�������ʼ��ʱע��
static void uarts_data_handler(ble_uarts_evt_t * p_evt)
{
  uint8_t cmd_data[20]={0};
  //�ж��¼�����:���յ��������¼�
  if(p_evt->type == BLE_UARTS_EVT_RX_DATA)
  {
    //���ڴ�ӡ�����յ�����
    for(uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
    {
      cmd_data[i] = p_evt->params.rx_data.p_data[i];
    }
    if(cmd_data[0] == 'c' && cmd_data[1] == 'l' && cmd_data[2] == 'e' && cmd_data[3] == 'a' && cmd_data[4] == 'r')
    {
      Cmd_Flag = 2;
    }
    if(cmd_data[0] == 's' && cmd_data[1] == 'e' && cmd_data[2] == 't') //�궨����
    {
      if(cmd_data[3] == '0') //�궨����
      {
        Set_Data = 0;
        Cmd_Flag = 1;
      }
      if(cmd_data[3] > '0' && cmd_data[3] <= '9' && cmd_data[4] == 0) //�궨����һλ����
      {
        Set_Data = (cmd_data[3] - '0')*1;
        Cmd_Flag = 1;
      }
      if(cmd_data[3] > '0' && cmd_data[3] <= '9' && cmd_data[4] >= '0' && cmd_data[4] <= '9' && cmd_data[5] == 0) //�궨������λ����
      {
        Set_Data = (cmd_data[3] - '0')*10 + (cmd_data[4] - '0');
        Cmd_Flag = 1;
      }
      if(cmd_data[3] > '0' && cmd_data[3] <= '9' && cmd_data[4] >= '0' && cmd_data[4] <= '9' && cmd_data[5] >= '0' && cmd_data[5] <= '9' && cmd_data[6] == 0) //�궨������λ����
      {
        Set_Data = (cmd_data[3] - '0')*100 + (cmd_data[4] - '0')*10 + (cmd_data[5] - '0')*1;
        Cmd_Flag = 1;
      }
    }
    if(cmd_data[0] == ':' && cmd_data[1] == '1' && cmd_data[2] == 'A' && cmd_data[3] == '1' && cmd_data[4] == '4') //��ȡ����
    {
      Ack_Flag = 1;
    }
    if(cmd_data[0] == ':' && cmd_data[1] == '1' && cmd_data[2] == 'A' && cmd_data[3] == '3' && cmd_data[4] == '4') //��ȡ�豸��Ϣ
    {
      Ack_Flag = 20;
    }
    if(cmd_data[0] == ':' && cmd_data[1] == '1' && cmd_data[2] == 'A' && cmd_data[3] == '5' && cmd_data[4] == '4') //��ȡ�汾
    {
      Ack_Flag = 21;
    }
    if(p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
    {
      while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
  }
}
//�����ʼ��,������ʼ���Ŷ�д��ģ��ͳ�ʼ��Ӧ�ó���ʹ�õķ���
static void services_init(void)
{
  ret_code_t         err_code;
  //���崮��͸����ʼ���ṹ��
  ble_uarts_init_t     uarts_init;
  //�����Ŷ�д���ʼ���ṹ�����
  nrf_ble_qwr_init_t  qwr_init = {0};
  //�Ŷ�д���¼�������
  qwr_init.error_handler = nrf_qwr_error_handler;
  //��ʼ���Ŷ�д��ģ��
  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  //��麯������ֵ
  APP_ERROR_CHECK(err_code);
  /*------------------���´����ʼ������͸������-------------*/
  //���㴮��͸�������ʼ���ṹ��
  memset(&uarts_init, 0, sizeof(uarts_init));
  //���ô���͸���¼��ص�����
  uarts_init.data_handler = uarts_data_handler;  // BLE receive callback
  //��ʼ������͸������
  err_code = ble_uarts_init(&m_uarts, &uarts_init);
  APP_ERROR_CHECK(err_code);
  /*------------------��ʼ������͸������-END-----------------*/
}

//���Ӳ���Э��ģ���¼�������
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  //�ж��¼�����,�����¼�����ִ�ж���
  //���Ӳ���Э��ʧ��,�Ͽ���ǰ����
  if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
  {
    NRF_LOG_INFO("CONN Failed");
  }
  //���Ӳ���Э�̳ɹ�
  if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
  {
  //���ܴ���;
  }
}

//���Ӳ���Э��ģ��������¼�,����nrf_error�����˴������,ͨ��nrf_error���Է���������Ϣ
static void conn_params_error_handler(uint32_t nrf_error)
{
  //���������
  NRF_LOG_INFO("App Reset 3");
  NVIC_SystemReset();
  APP_ERROR_HANDLER(nrf_error);
}

//���Ӳ���Э��ģ���ʼ��
static void conn_params_init(void)
{
  ret_code_t             err_code;
  //�������Ӳ���Э��ģ���ʼ���ṹ��
  ble_conn_params_init_t  cp_init;
  //����֮ǰ������
  memset(&cp_init, 0, sizeof(cp_init));
  //����ΪNULL,��������ȡ���Ӳ���
  cp_init.p_conn_params                  = NULL;
  //���ӻ�����֪ͨ���״η������Ӳ�����������֮���ʱ������Ϊ5��
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  //ÿ�ε���sd_ble_gap_conn_param_update()�����������Ӳ������������֮��ļ��ʱ������Ϊ:30��
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  //�������Ӳ���Э��ǰ�������Ӳ���Э�̵�����������Ϊ:3��
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  //���Ӳ������´������¼���ʼ��ʱ
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  //���Ӳ�������ʧ�ܲ��Ͽ�����
  cp_init.disconnect_on_fail             = false;
  //ע�����Ӳ��������¼����
  cp_init.evt_handler                    = on_conn_params_evt;
  //ע�����Ӳ������´����¼����
  cp_init.error_handler                  = conn_params_error_handler;
  //���ÿ⺯���������Ӳ������³�ʼ���ṹ��Ϊ�����������ʼ�����Ӳ���Э��ģ��
  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}
//BLE�¼�������
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  ret_code_t err_code = NRF_SUCCESS;
  ble_gatt_params_on_ble_evt(p_ble_evt);
  //�ж�BLE�¼�����,�����¼�����ִ����Ӧ����
  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_DISCONNECTED://�Ͽ������¼�
         m_conn_handle = BLE_CONN_HANDLE_INVALID;
         NRF_LOG_INFO("Disconnected. reason: 0x%04x",p_ble_evt->evt.gap_evt.params.disconnected.reason);
         if(Sys_State == 2)
         {
           Sys_State = 1;
           Set_Led(Sys_State);
         }
         //��ӡ��ʾ��Ϣ
         NRF_LOG_INFO("BLE Breaked!");
         Ack_Flag = 0;     //��շ��Ͷ���
         Cmd_Flag = 0;     //����������
         break;
    case BLE_GAP_EVT_CONNECTED://�����¼�
         NRF_LOG_INFO("BLE Connected!");
         nrf_gpio_pin_set(SENSOR_PW);
         Ack_Flag = 0;     //��շ��Ͷ���
         Cmd_Flag = 0;     //����������
         Sys_State = 2;
         //�������Ӿ��
         m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
         //�����Ӿ��������Ŷ�д��ʵ��,������Ŷ�д��ʵ���͸����ӹ���,����,���ж�����ӵ�ʱ��,ͨ��������ͬ���Ŷ�д��ʵ��,�ܷ��㵥�������������
         err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
         APP_ERROR_CHECK(err_code);
         //�������ӵķ��书��
         //nRF52832�����õ�ֵΪ:4,3,0,-4,-8,-12,-16,-20,-30,-40
         //nRF52840�����õ�ֵΪ:8,7,6,5,4,3,2,0,-4,-8,-12,-16,-20,-30,-40
         err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, BLE_CONN_PW_LEVEL);
         APP_ERROR_CHECK(err_code);
         break;
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST://PHY�����¼�
         {
           ble_gap_phys_t const phys =
           {
             .rx_phys = BLE_GAP_PHY_AUTO,
             .tx_phys = BLE_GAP_PHY_AUTO,
           };
           //��ӦPHY���¹��
           err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
           APP_ERROR_CHECK(err_code);
         }
         break;
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST://��ȫ���������¼�
         //��֧�����
         err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
         APP_ERROR_CHECK(err_code);
    case BLE_GATTS_EVT_SYS_ATTR_MISSING://ϵͳ���Է������ڵȴ���
         //ϵͳ����û�д洢,����ϵͳ����
         err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
         APP_ERROR_CHECK(err_code);
         break;
    case BLE_GATTC_EVT_TIMEOUT://GATT�ͻ��˳�ʱ�¼�
         NRF_LOG_INFO("GATTC Timeout");
         break;
    case BLE_GATTS_EVT_TIMEOUT://GATT��������ʱ�¼�
         NRF_LOG_INFO("GATTS Timeout");
         break;
    default:
         break;
  }
}

//��ʼ��BLEЭ��ջ
static void ble_stack_init(void)
{
  ret_code_t  err_code;
  //����ʹ��SoftDevice,�ú����л����sdk_config.h�ļ��е�Ƶʱ�ӵ����������õ�Ƶʱ��
  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);
  //���屣��Ӧ�ó���RAM��ʼ��ַ�ı���
  uint32_t ram_start = 0;
  //ʹ��sdk_config.h�ļ���Ĭ�ϲ�������Э��ջ,��ȡӦ�ó���RAM��ʼ��ַ,���浽����ram_start
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);
  //ʹ��BLEЭ��ջ
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);
  //ע��BLE�¼��ص�����
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

//��ʼ����Դ����ģ��
static void power_management_init(void)
{
  ret_code_t  err_code;
  //��ʼ����Դ����
  err_code = nrf_pwr_mgmt_init();
  //��麯�����صĴ������
  APP_ERROR_CHECK(err_code);
}
//��ʼ��APP��ʱ��ģ��
static void timers_init(void)
{
  //��ʼ��APP��ʱ��ģ��
  ret_code_t  err_code = app_timer_init();
  //��鷵��ֵ
  APP_ERROR_CHECK(err_code);
  //�����û���ʱ���� 
  err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, m_app_timeout_handler); // callback
  APP_ERROR_CHECK(err_code);
  //����������ʱ����
  err_code = app_timer_create(&m_avg_timer_id, APP_TIMER_MODE_REPEATED, m_avg_timeout_handler); // callback
  APP_ERROR_CHECK(err_code);
  //�����û���ʱ����
  err_code = app_timer_start(m_app_timer_id, APP_TIMER_INTERVAL, NULL); // time
  APP_ERROR_CHECK(err_code);
  //����������ʱ����
  err_code = app_timer_start(m_avg_timer_id, AVG_TIMER_INTERVAL, NULL); // time
  APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
  //��ʼ��log����ģ��
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
  //����log����նˣ�����sdk_config.h�е�������������ն�ΪUART����RTT��
  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//����״̬�����������û�й������־����,��˯��ֱ����һ���¼���������ϵͳ
static void idle_state_handle(void)
{
  //��������log
  if(NRF_LOG_PROCESS() == false)
  {
    //���е�Դ����,�ú�����Ҫ�ŵ���ѭ������ִ��
    nrf_pwr_mgmt_run();
  }
}

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)

short  off_sta = 0;

void  button_event_handler(uint8_t pin_no, uint8_t button_action)//APP�����¼��ص�����
{
  if(off_sta == -1)  return;
	
  if(pin_no == POW_BTN)//�жϼ�ֵ
  {
    if(button_action == APP_BUTTON_PUSH)   //���������¼�
    {
	  /*
      if(Sys_State == 0 && Charge_State == 4)
      {
        Sys_State = 1;
        Set_Led(Sys_State);
        ble_adv_start();
        Sleep_Count = 0;
      }
	  */
      if(Sys_State > 0)
      {
        Sys_State = 0;
        Set_Led(Sys_State);
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        ble_adv_stop();
		return;
      }
	  return;
    }	
	
    if(button_action == APP_BUTTON_RELEASE)    {
      if(Sys_State == 0)      {
		/*zj.huang 20230711*/
	    deep_sleep();
        //nrf_gpio_pin_clear(POW_OUT1);
        //nrf_gpio_pin_clear(POW_OUT2);
      }
    }
	
  }
}

static void buttons_init(void)
{
  ret_code_t  err_code;
  //app����������붨��Ϊstatic����,��Ϊ��������ģ����Ҫ����ָ��������ָ��
  static app_button_cfg_t  buttons[] =
  {
    {POW_BTN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
//    {CHG_ING, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
//    {CHG_IDLE,APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
  };
  //��ʼ��APP����
  err_code = app_button_init(buttons, ARRAY_SIZE(buttons),BUTTON_DETECTION_DELAY);
  APP_ERROR_CHECK(err_code);
  //ʹ��APP����
  err_code = app_button_enable();
  APP_ERROR_CHECK(err_code);
}

void IO_Init(void)
{
	/*zj.huang 20230711*/
	nrf_gpio_cfg_output(POW_OUT1);
//	nrf_gpio_cfg_output(POW_OUT2);  // important
	nrf_gpio_cfg_output(SENSOR_PW);
	
	/*zj.huang 20230719*/
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 27));  // config mux
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 26));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 25));
	//nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0, 27));

    
	/*zj.huang 20230818*/
	//nrf_delay_ms(20);
    
    //nrf_gpio_pin_set(POW_OUT1);
//    nrf_gpio_pin_set(POW_OUT2);

  nrf_gpio_cfg_input(AD_DOUT, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_output(AD_SCK);

  nrf_gpio_cfg_output(LED_RD);
  nrf_gpio_cfg_output(LED_GN);

  //nrf_gpio_pin_clear(SENSOR_PW);
  
    
    /*zj.huang 20230809*/
    nrf_gpio_cfg_input(CHG_ING,  NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(CHG_IDLE,  NRF_GPIO_PIN_PULLUP); // force pull HIGH
              
    for(short i = 0; i < 2; ++i) {
        nrf_gpio_pin_set(LED_GN);
        nrf_delay_ms(20);
        nrf_gpio_pin_clear(LED_GN);
        nrf_delay_ms(20);
        nrf_gpio_pin_set(LED_RD);
        nrf_delay_ms(20);
        nrf_gpio_pin_clear(LED_RD);
        nrf_delay_ms(20);
    }
    //-------------------------------------------------------
    nrf_gpio_pin_set(POW_OUT1);
    nrf_delay_ms(20);
    
    nrf_gpio_pin_clear(SENSOR_PW);
    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0, 27));
    
    
    Sys_State = 1;
    Set_Led(Sys_State);
}

void wdt_event_handler(void)
{
  NRF_LOG_INFO("Watch Dog!");
}

//���Ź���ʼ��,��ʼ����ɺ���������Ź�,���Ź�һ����������޷�ֹͣ
void wdt_init(void)
{
  uint32_t  err_code = NRF_SUCCESS;
  //����WDT���ýṹ�岢ʹ��
  nrfx_wdt_config_t  config = NRFX_WDT_DEAFULT_CONFIG;
  //��ʼ��WDT
  err_code = nrfx_wdt_init(&config, wdt_event_handler);
  APP_ERROR_CHECK(err_code);
  //����ι��ͨ��,Ҳ����ʹ���ĸ�
  err_code = nrfx_wdt_channel_alloc(&m_channel_id);
  APP_ERROR_CHECK(err_code);
  //����WDT
  nrfx_wdt_enable();       
}

//SAADC�¼��ص�����,��Ϊ�Ƕ���ģʽ,���Բ���Ҫ�¼�,���ﶨ����һ���յ��¼��ص�����
void saadc_callback(nrf_drv_saadc_evt_t const * p_event){}

//��ʼ��SAADC,����ʹ�õ�SAADCͨ���Ĳ���
void saadc_init(void)
{
  ret_code_t  err_code;
  //����ADCͨ�����ýṹ�壬��ʹ�õ��˲������ú��ʼ����
  //NRF_SAADC_INPUT_AIN2��ʹ�õ�ģ������ͨ��
  nrf_saadc_channel_config_t  channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
  //��ʼ��SAADC��ע���¼��ص�������
  err_code = nrf_drv_saadc_init(NULL, saadc_callback);
  APP_ERROR_CHECK(err_code);
  //��ʼ��SAADCͨ��0
  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);
}


void  unused_IO(void)
{
	///*
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 0));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 1));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 2));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 3));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 7));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 8));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 11));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 13));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 17));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 19));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 22));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 23));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 24));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 29));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 31));
	
	nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 0));
	nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 1));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 2));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 3));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 7));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 8));
	nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 11));
	nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 13));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 17));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 19));
	nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 22));
	nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 23));
	nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 24));
	nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 29));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 31));
	//*/
}

//������
int main(void)
{
  /*zj.huang 20230711*/
  unused_IO();
	
  //��ʼ��IO
  IO_Init();
	
  //��ʼ�����Ź�
  wdt_init();
	
  //��ʼ������
//  uart_config();	
  //��ʼ��SAADC
  saadc_init();   		
  //��ʼ������
  buttons_init();	
	
  //��ʼ��FDS,���ڽ��豸���Ʊ��浽Ƭ��Flash
  my_fds_init();
	
  //��ʼ��log����ģ��
//  log_init();
	
  //��ʼ��APP��ʱ��
  timers_init();

	
  //��ʼ����Դ����
  power_management_init();	
  //��ʼ��Э��ջ
  ble_stack_init();	
	
  //��ʼ��GAP����
  gap_params_init();  
  //��ʼ��GATT
  gatt_init();  
  //��ʼ������
  services_init();  
  //��ʼ���㲥
  advertising_init();  
  //��ʼ�����Ӳ���Э��
  conn_params_init();  
    
  //�����㲥
  advertising_start();


  //��ѭ��
//  NRF_LOG_INFO("Initial Finish!");  
  
  //����SWD��
  //Lock_Data();
  
  while(true)
  {
    //������������
    weight_data_handle();
    //�豸���Ʊ���
    device_name_handle();
    //��������LOG�����е�Դ����
    idle_state_handle();
  }
}


void  deep_sleep(void)
{
	Sys_State = 0;
	Batt_samp_n = 0;
	
	app_timer_stop_all();
	nrf_delay_ms(50);
	
	sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	nrf_delay_ms(50);
    ble_adv_stop();
	nrf_delay_ms(50);
	
	nrfx_saadc_uninit();
	nrf_delay_ms(50);
	app_uart_close();
	nrf_delay_ms(50);

	
	nrf_gpio_pin_set(LED_RD);
    nrf_gpio_pin_set(LED_GN);
	nrf_delay_ms(100);
	nrf_gpio_pin_clear(LED_RD);
    nrf_gpio_pin_clear(LED_GN);
	nrf_delay_ms(100);
	nrf_gpio_pin_set(LED_RD);
    nrf_gpio_pin_set(LED_GN);
	nrf_delay_ms(100);
	nrf_gpio_pin_clear(LED_RD);
    nrf_gpio_pin_clear(LED_GN);
	nrf_delay_ms(100);
    nrf_gpio_pin_set(LED_RD);
    nrf_gpio_pin_set(LED_GN);
	nrf_delay_ms(100);
	nrf_gpio_pin_clear(LED_RD);
    nrf_gpio_pin_clear(LED_GN);
	nrf_delay_ms(100);


	off_sta = -1;
	
	nrf_gpio_pin_clear( NRF_GPIO_PIN_MAP(0,12) );
	
//	nrf_gpio_pin_clear( NRF_GPIO_PIN_MAP(0,14) );
    
    nrf_gpio_pin_clear( NRF_GPIO_PIN_MAP(0,30) );
	//nrf_gpio_pin_clear(POW_OUT1);
    //nrf_gpio_pin_clear(POW_OUT2);
		
	//nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);	
}

