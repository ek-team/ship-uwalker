//���õ�C��ͷ�ļ�
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//Log��Ҫ���õ�ͷ�ļ�
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_gap.h"
//����FDSͷ�ļ�
#include "fds.h"
#include "device_name_op.h"
//�㲥��Ҫ���õ�ͷ�ļ�
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "bsp_btn_ble.h"
//����͸����Ҫ���õ�ͷ�ļ�
#include "my_ble_uarts.h"

#define DEFAULT_DEVICE_NAME             "Walker_UH"                   //�豸�����ַ���
#define APP_ADV_INTERVAL                320                           //�㲥��� (200ms),��λ0.625 ms 
#define APP_ADV_DURATION                0                             //�㲥����ʱ��,��λ:10ms������Ϊ0��ʾ����ʱ
#define UARTS_SERVICE_UUID_TYPE         BLE_UUID_TYPE_VENDOR_BEGIN    //����͸������UUID����:�����Զ���UUID

BLE_ADVERTISING_DEF(m_advertising);                                   //��������Ϊm_advertising�Ĺ㲥ģ��ʵ��
#define BLE_ADV_PW_LEVEL                4                             //�㲥���书��
extern int16_t NO_Data;

void ble_adv_start(void)
{
  ble_adv_modes_config_t config;
  config.ble_adv_fast_enabled = true;
  config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  config.ble_adv_fast_timeout = APP_ADV_DURATION;
  config.ble_adv_on_disconnect_disabled = false;
  ble_advertising_modes_config_set(&m_advertising,&config);

  NRF_LOG_INFO("BLE Adv Start!%d",ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST));
}

void ble_adv_stop(void)
{
  ble_adv_modes_config_t config;
  config.ble_adv_fast_enabled = true;
  config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  config.ble_adv_fast_timeout = APP_ADV_DURATION;
  config.ble_adv_on_disconnect_disabled = true;
  ble_advertising_modes_config_set(&m_advertising,&config);
	
  sd_ble_gap_adv_stop(m_advertising.adv_handle);
  NRF_LOG_INFO("BLE Adv Stop!%d",ble_advertising_start(&m_advertising, BLE_ADV_MODE_IDLE));
}

//���崮��͸������UUID�б�
static ble_uuid_t m_adv_uuids[]          =                                          
{
  {BLE_UUID_UARTS_SERVICE, UARTS_SERVICE_UUID_TYPE}
};

//�����豸���Ʋ����ṹ�����
static device_name_info_t device_name_info;

//�����������ݲ����ṹ�����
weight_data_info_t weight_data_info;

//����FDS�첽������־�ṹ�����
static my_fds_info_t my_fds_info;

//�����豸��Ϣ
static device_name_t device_name;

//������������
static weight_data_t weight_data;

//�����豸������Ϣ�ļ�¼
static fds_record_t const device_name_record =
{
  .file_id           = NAME_FILE,
  .key               = NAME_REC_KEY,
  .data.p_data       = &device_name,
  //��¼�ĳ��ȱ�����4�ֽڣ��֣�Ϊ��λ
  .data.length_words = (sizeof(device_name) + 3) / sizeof(uint32_t),
};

//��������������Ϣ�ļ�¼
static fds_record_t const weight_data_record =
{
  .file_id           = DATA_FILE,
  .key               = DATA_REC_KEY,
  .data.p_data       = &weight_data,
  //��¼�ĳ��ȱ�����4�ֽڣ��֣�Ϊ��λ
  .data.length_words = (sizeof(weight_data) + 3) / sizeof(uint32_t),
};

//FDS�¼�������
static void fds_evt_handler(fds_evt_t const * p_evt)
{
  //�ж��¼�����
  switch (p_evt->id)
  {
    case FDS_EVT_INIT://FDS��ʼ���¼�
         if(p_evt->result == NRF_SUCCESS)
         {
           NRF_LOG_INFO("FDS init successfully! ");
           my_fds_info.busy = false;
         }
         break;
    case FDS_EVT_WRITE://FDSд��¼�¼�
				 if(p_evt->result == NRF_SUCCESS)
         {
           my_fds_info.busy = false;
         }
				 break;
    case FDS_EVT_UPDATE://FDS���¼�¼�¼�
         if (p_evt->result == NRF_SUCCESS)
         {
           my_fds_info.busy = false;
         }
				 break;
    case FDS_EVT_GC://FDS��Ƭ�����¼�
				 if(p_evt->result == NRF_SUCCESS)
         {
           my_fds_info.busy = false;
         }
         break;
         default:
         break;
  }
}
	
//�ȴ�FDS��ʼ�����
static void wait_for_fds_ready(void)
{
  while(my_fds_info.busy)
  {
    (void)sd_app_evt_wait();
  }
}

ret_code_t my_fds_init (void)
{
  //ע��FDS�¼��ص���������FdS�¼�,�ڵ���fds_init()����֮ǰ,һ��Ҫ��ע��
  ret_code_t ret = fds_register(fds_evt_handler);
  if(ret != NRF_SUCCESS)
  {
    return ret;	
  }
  my_fds_info.busy = true;
  ret = fds_init();//��ʼ��FDS
  if (ret != NRF_SUCCESS)
  {
    return ret;
  }
  //FDS��ʼ�����첽��,���Ҫ�ȴ�FDS��ʼ�����
  wait_for_fds_ready();
  return NRF_SUCCESS;		
}

static ret_code_t fds_save_devicename (void)
{
  ret_code_t              err_code;
	//���岢��ʼ����¼�������ṹ�����
  fds_record_desc_t desc = {0};	
  //���岢��ʼ����¼�������ƽṹ�����
  fds_find_token_t  tok  = {0};
  //����tok,��ͷ����
  memset(&tok, 0x00, sizeof(fds_find_token_t));
  //��DEVICE_FILE�ļ��в��Ҽ�¼m_desp_record
  err_code = fds_record_find(NAME_FILE, NAME_REC_KEY, &desc, &tok);	
  
  memcpy(&device_name, &device_name_info.name, sizeof(device_name));
	
	//û�в��ҵ�m_desp_record��¼,д���¼
  if(err_code != NRF_SUCCESS)
  {
    my_fds_info.busy = true;//д��¼֮ǰ��λFDSæ��־
    err_code = fds_record_write(&desc, &device_name_record);
    //���ݺ�������ֵ�ж��Ƿ���Ҫ��Ƭ�ռ�
    if(err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
      my_fds_info.gc = true;
      my_fds_info.busy = false;
    }
    else
    {
      APP_ERROR_CHECK(err_code);
      wait_for_fds_ready();
    }
    NRF_LOG_INFO("write");
  }
	else//�豸���Ƽ�¼�Ѿ�����,���¼�¼
	{
    my_fds_info.busy = true;//���¼�¼֮ǰ��λFDSæ��־
    err_code = fds_record_update(&desc, &device_name_record);
    //���ݺ�������ֵ�ж��Ƿ���Ҫ��Ƭ�ռ�
    if(err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
      my_fds_info.gc = true;//��Ƭ�ռ���־��λ
      my_fds_info.busy = false;//FDSæ��־����
    }
    else
    {
      APP_ERROR_CHECK(err_code);
      wait_for_fds_ready();
    }
  }
  return NRF_SUCCESS;
}

static ret_code_t fds_save_weightdata (void)
{
	ret_code_t              err_code;
  //���岢��ʼ����¼�������ṹ�����
  fds_record_desc_t desc = {0};	
  //���岢��ʼ����¼�������ƽṹ�����
  fds_find_token_t  tok  = {0};
  //����tok,��ͷ����
  memset(&tok, 0x00, sizeof(fds_find_token_t));
	//��DEVICE_FILE�ļ��в��Ҽ�¼m_desp_record
  err_code = fds_record_find(DATA_FILE, DATA_REC_KEY, &desc, &tok);
  memcpy(&weight_data, &weight_data_info.data, sizeof(weight_data));
  //û�в��ҵ�m_desp_record��¼,д���¼
  if(err_code != NRF_SUCCESS)
  {
    my_fds_info.busy = true;//д��¼֮ǰ��λFDSæ��־
    err_code = fds_record_write(&desc, &weight_data_record);
    //���ݺ�������ֵ�ж��Ƿ���Ҫ��Ƭ�ռ�
    if(err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
      my_fds_info.gc = true;
      my_fds_info.busy = false;
		}
		else
		{
      APP_ERROR_CHECK(err_code);
      wait_for_fds_ready();
    }
    NRF_LOG_INFO("write");
  }
  else//�豸���Ƽ�¼�Ѿ�����,���¼�¼
  {
    my_fds_info.busy = true;//���¼�¼֮ǰ��λFDSæ��־
    err_code = fds_record_update(&desc, &weight_data_record);
    //���ݺ�������ֵ�ж��Ƿ���Ҫ��Ƭ�ռ�
    if(err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
      my_fds_info.gc = true;//��Ƭ�ռ���־��λ
      my_fds_info.busy = false;//FDSæ��־����
    }
    else
    {
      APP_ERROR_CHECK(err_code);
      wait_for_fds_ready();
    }
    NRF_LOG_INFO("flash");
  }
  return NRF_SUCCESS;
}

//��Flash�в����Ƿ������Ч���豸����
static uint32_t find_device_name(void)
{
  uint32_t *data;
  ret_code_t rc = FDS_ERR_NOT_FOUND;
  //���岢��ʼ����¼�������ṹ�����
  fds_record_desc_t desc = {0};
  //���岢��ʼ����¼�������ƽṹ�����
  fds_find_token_t  tok  = {0};
  //����tok,��ͷ����
  memset(&tok, 0x00, sizeof(fds_find_token_t));
  //��DEVICE_FILE�ļ��в��Ҽ�¼m_version_record
  rc = fds_record_find(NAME_FILE, NAME_REC_KEY, &desc, &tok);
  //���ҵ���¼��,��ȡ��¼����
  if(rc == NRF_SUCCESS)
  {
    fds_flash_record_t temp = {0};
    //�򿪼�¼��ȡ��¼����
		rc = fds_record_open(&desc, &temp);
		APP_ERROR_CHECK(rc);
		data = (uint32_t *) temp.p_data;
    //��ȡ�豸����
    for (uint8_t i=0;i<temp.p_header->length_words;i++)
    {
      device_name_info.name.name32[i] = data[i];
    }				
    //��ȡ��,�رռ�¼
    rc = fds_record_close(&desc);
    APP_ERROR_CHECK(rc);
  }
  return rc;
}

//��Flash�в����Ƿ������Ч����������
static uint32_t find_weight_data(void)
{
  uint32_t *data;
  ret_code_t rc = FDS_ERR_NOT_FOUND;
  //���岢��ʼ����¼�������ṹ�����
  fds_record_desc_t desc = {0};
  //���岢��ʼ����¼�������ƽṹ�����
  fds_find_token_t  tok  = {0};
  //����tok,��ͷ����
  memset(&tok, 0x00, sizeof(fds_find_token_t));
  //��DEVICE_FILE�ļ��в��Ҽ�¼m_version_record
  rc = fds_record_find(DATA_FILE, DATA_REC_KEY, &desc, &tok);
  //���ҵ���¼��,��ȡ��¼����
  if(rc == NRF_SUCCESS)
  {
    fds_flash_record_t temp = {0};
    //�򿪼�¼��ȡ��¼����
    rc = fds_record_open(&desc, &temp);
    APP_ERROR_CHECK(rc);
		data = (uint32_t *) temp.p_data;
    //��ȡ�豸����
    for(uint8_t i=0;i<temp.p_header->length_words;i++)
    {
      weight_data_info.data.data32[i] = data[i];
    }				
    //��ȡ��,�رռ�¼
    rc = fds_record_close(&desc);
    APP_ERROR_CHECK(rc);
  }
  return rc;
}

//����GAP�豸����:���Flash���Ѿ��洢���豸����,�������ƽ�������,����ʹ��Ĭ�ϵ�����
void device_name_set(void)
{
  ret_code_t               err_code;
  ble_gap_conn_sec_mode_t  sec_mode;
	
  //����GAP�İ�ȫģʽ
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode); // 1-1 open mode
	
  //Flash�в��ҵ�����Ч���豸����,��ʹ�ø��豸����
  if(find_device_name() == NRF_SUCCESS)
  {
    if(device_name_info.name.name32[0] == NAME_FLASH_TYPE_NAME)
    {
      err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)(&device_name_info.name.name8[8]), device_name_info.name.name32[1]);
      APP_ERROR_CHECK(err_code);
      device_name_info.name_exist = true;      //��־��λ,��ʾFlash���Ѿ��洢���豸���ơ��ٴ��޸ĵ�ʱ����¼��ɶ���������д
    }	
    else device_name_info.name_exist = false;
  }
  //Flash��û�в��ҵ�����Ч���豸����,ʹ��Ĭ�ϵ��豸����
  if(device_name_info.name_exist == false)
  {
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEFAULT_DEVICE_NAME, strlen(DEFAULT_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
  }
}

//������������:���Flash���Ѿ��洢����������,����������������,����ʹ��Ĭ�ϵ�����
void weight_data_set(void)
{
	ble_gap_conn_sec_mode_t sec_mode;
  //����GAP�İ�ȫģʽ
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	
	//Flash�в��ҵ�����Ч����������,��ʹ�ø���������
	if(find_weight_data() == NRF_SUCCESS)
	{
		if(weight_data_info.data.data32[0] == DATA_FLASH_TYPE_NAME)
		{
      //��ȡ����
			weight_data_info.data_exist = true;      //��־��λ,��ʾFlash���Ѿ��洢���������ݡ��ٴ��޸ĵ�ʱ����¼��ɶ���������д
		}	
		else weight_data_info.data_exist = false;
	}
	//Flash��û�в��ҵ�����Ч����������,ʹ��Ĭ�ϵ���������
	if(weight_data_info.data_exist == false)
	{
    //��ȡ����ʱʹ��Ĭ������
		NO_Data = 1;
	}
}
//���¹㲥����
static void adv_updata(void)
{
	ret_code_t err_code;
	ble_advdata_t           adv_data; //�㲥����
  ble_advdata_t           sr_data;  //ɨ����Ӧ����
	
	//����֮ǰ������
	memset(&adv_data, 0, sizeof(adv_data));
	memset(&sr_data, 0, sizeof(sr_data));
	//�����豸����:ȫ��
	adv_data.name_type               = BLE_ADVDATA_FULL_NAME;
	//�Ƿ�������:����
  adv_data.include_appearance      = false;
	//Flag:һ��ɷ���ģʽ,��֧��BR/EDR
  adv_data.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
			
  //UUID�ŵ�ɨ����Ӧ����
  sr_data.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  sr_data.uuids_complete.p_uuids  = m_adv_uuids;
  //���¹㲥����
  err_code = ble_advertising_advdata_update(&m_advertising, &adv_data, &sr_data);
  APP_ERROR_CHECK(err_code);
}
void ble_gatt_params_on_ble_evt(ble_evt_t const * p_ble_evt)
{
	//����һ��GATTSд�¼��ṹ�岢ָ��BLE�¼��ṹ���е�д�¼�,����ȡֵ
	ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  //����¼�������BLE_GATTS_EVT_WRITE����UIID��GAP������豸����������UUID,��ΪдGAP�豸��������
	if((p_evt_write->uuid.uuid == BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME) && (p_ble_evt->header.evt_id == BLE_GATTS_EVT_WRITE))
  {
    //�����豸���� 
    memcpy(&device_name_info.name.name8[8], p_evt_write->data, p_evt_write->len);
    //��һ��������Ϊ��������
    device_name_info.name.name32[0] = NAME_FLASH_TYPE_NAME;
    //�ڶ���������Ϊ���Ƶĳ���
    device_name_info.name.name32[1] = p_evt_write->len;
    //���¹㲥����
    adv_updata();
    //�豸���Ʊ����־��λ,��ѭ���и��ݸñ�־�ж��Ƿ���Ҫ���豸���ƴ洢��Flash
    device_name_info.save_name_flag = true; 
  }	
}

//�㲥�¼�������
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  //�жϹ㲥�¼�����
  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_FAST://���ٹ㲥�����¼�:���ٹ㲥�������������¼�
         NRF_LOG_INFO("Fast advertising.");
         break;
    case BLE_ADV_EVT_IDLE://�㲥IDLE�¼�:�㲥��ʱ���������¼�
         break;
    default:
         break;
  }
}
//�㲥��ʼ��
void advertising_init(void)
{
  ret_code_t             err_code;
  //���屣�淢�书�ʵȼ��ı���
  int8_t  tx_power_level = BLE_ADV_PW_LEVEL;
  //����㲥��ʼ�����ýṹ�����
  ble_advertising_init_t init;
  //����֮ǰ������
  memset(&init, 0, sizeof(init));
  //�豸��������:ȫ��
  init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
  //�Ƿ�������:����
  init.advdata.include_appearance      = false;
  //���͹��ʵȼ�
  init.advdata.p_tx_power_level = &tx_power_level;
  //Flag:һ��ɷ���ģʽ,��֧��BR/EDR
  init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  //UUID�ŵ�ɨ����Ӧ����
  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
	
  //���ù㲥ģʽΪ���ٹ㲥
  init.config.ble_adv_fast_enabled  = true;
  //���ù㲥����͹㲥����ʱ��
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
  //�㲥�¼��ص�����
  init.evt_handler = on_adv_evt;
  //��ʼ���㲥
  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);
  //���ù㲥���ñ�ǡ�APP_BLE_CONN_CFG_TAG�����ڸ��ٹ㲥���õı��,����Ϊδ��Ԥ����һ������,�ڽ�����SoftDevice�汾��,
  //����ʹ��sd_ble_gap_adv_set_configure()�����µĹ㲥����
  //��ǰSoftDevice�汾��S112 V7.2.0�汾��֧�ֵ����㲥������Ϊ1,���APP_BLE_CONN_CFG_TAGֻ��д1��
  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
  //���ù㲥�ķ��书��
  //nRF52832�����õ�ֵΪ:4,3,0,-4,-8,-12,-16,-20,-30,-40
  //nRF52840�����õ�ֵΪ:8,7,6,5,4,3,2,0,-4,-8,-12,-16,-20,-30,-40
  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, BLE_ADV_PW_LEVEL);
  APP_ERROR_CHECK(err_code);
}
//�����㲥,�ú������õ�ģʽ����͹㲥��ʼ�������õĹ㲥ģʽһ��
void advertising_start(void)
{
  //ʹ�ù㲥��ʼ�������õĹ㲥ģʽ�����㲥
  ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  //��麯�����صĴ������
  APP_ERROR_CHECK(err_code);
}
//�����豸���ƴ洢����Ƭ�ռ�
void device_name_handle(void)
{
  ret_code_t rc;
  if(my_fds_info.gc == true)//��Ƭ�ռ���־��λ,��ʾFDS�ļ�¼����,��Ҫִ����Ƭ�ռ�
  {
    if(my_fds_info.busy == false)
    {
      my_fds_info.busy = true;
      rc = fds_gc();//FDS��Ƭ����
      APP_ERROR_CHECK(rc);//�ô�����ģ���麯������ֵ
      wait_for_fds_ready();
				
      my_fds_info.gc = false;
      //��Ƭ�ռ���ɺ�,�������豸���Ʊ�־��λ,���汾�ε��豸����
      device_name_info.save_name_flag = true;
    }
  }
  else//�洢�豸���Ʊ�־��λ,���豸���ƴ洢��Flash
  {
    if(device_name_info.save_name_flag == true)
    {
      if(my_fds_info.busy == false)
      {
        //д/�����豸���Ƽ�¼
        fds_save_devicename();
        //�洢�豸���Ʊ�־����
        device_name_info.save_name_flag = false;
      }
    }
  }
}
//�����������ݴ洢����Ƭ�ռ�
void weight_data_handle(void)
{
	ret_code_t rc;
	if(my_fds_info.gc == true)//��Ƭ�ռ���־��λ,��ʾFDS�ļ�¼����,��Ҫִ����Ƭ�ռ�
	{
    if(my_fds_info.busy == false)
    {
      my_fds_info.busy = true;
      rc = fds_gc();//FDS��Ƭ����
      APP_ERROR_CHECK(rc);//�ô�����ģ���麯������ֵ
      wait_for_fds_ready();
      my_fds_info.gc = false;
      //��Ƭ�ռ���ɺ�,�������������ݱ�־��λ,���汾�ε���������
      weight_data_info.save_data_flag = true;
    }
  }
  else//�洢�������ݱ�־��λ,���������ݴ洢��Flash
  {
    if(weight_data_info.save_data_flag == true)
    {
      if(my_fds_info.busy == false)
      {
        //д/�����������ݼ�¼
        fds_save_weightdata();
        //�洢�������ݱ�־����
        weight_data_info.save_data_flag = false;
      }
    }
  }
}
