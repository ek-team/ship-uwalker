#ifndef DEVICE_NAME_OP_H__
#define DEVICE_NAME_OP_H__
//���õ�C��ͷ�ļ�
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ble.h"
//����FDSͷ�ļ�
#include "fds.h"

#define APP_BLE_CONN_CFG_TAG         1          //SoftDevice BLE���ñ�־
//�����ļ�ID�͸��ļ������ļ�¼��KEY
#define NAME_FILE                    (0x1000)   //�ļ�ID
#define NAME_REC_KEY                 (0x1001)   //��¼KEY,�ü�¼��ŵ��ļ�ID=0x1000
#define NAME_FLASH_TYPE_NAME         0x5A5A5A5A //������ʾ�ü�¼�洢�ǵ��豸����
#define NAME_FLASH_LEN_WORDS         10         //�豸���Ƴ���

//�����ļ�ID�͸��ļ������ļ�¼��KEY
#define DATA_FILE                    (0x2000)   //�ļ�ID
#define DATA_REC_KEY                 (0x2001)   //��¼KEY,�ü�¼��ŵ��ļ�ID=0x2000
#define DATA_FLASH_TYPE_NAME         0x6B6B6B6B //������ʾ�ü�¼�洢�ǵı궨����
#define DATA_FLASH_LEN_WORDS         10         //�궨���ݳ���

//�豸������Ϣ�ṹ��
typedef struct
{
  bool save_name_flag;                          //�豸���ƴ洢��־,��ѭ���м��ñ�־�Ƿ���λ,�Ӷ��ж��Ƿ����µ��豸������Ҫ�洢
  bool name_exist;                              //Flash���Ѿ��洢���豸����
  union
  {
    uint32_t name32[NAME_FLASH_LEN_WORDS];      //����豸���Ƶ�����,��������Ϊuint32_t��
    uint8_t name8[NAME_FLASH_LEN_WORDS*4];      //����豸���Ƶ�����,��������Ϊuint8_t�� 
  }name;
} device_name_info_t;

//����������Ϣ�ṹ��
typedef struct
{
  bool save_data_flag;                          //�������ݴ洢��־,��ѭ���м��ñ�־�Ƿ���λ,�Ӷ��ж��Ƿ����µ�����������Ҫ�洢
  bool data_exist;                              //Flash���Ѿ��洢����������
  union
  {
    uint32_t data32[DATA_FLASH_LEN_WORDS];      //����������ݵ�����,��������Ϊuint32_t��
    uint8_t data8[DATA_FLASH_LEN_WORDS*4];      //����������ݵ�����,��������Ϊuint8_t�� 
  }data;
} weight_data_info_t;

//FDS�첽������־�ṹ��
typedef struct
{
  bool gc;         //��Ƭ�ռ���־
  bool busy;       //FDSæ��־
}my_fds_info_t;

//��¼m_name_record������,ע��һ��Ҫ4�ֽڶ���
typedef struct
{
  uint8_t     name[NAME_FLASH_LEN_WORDS*4];    //�豸���ƴ洢�ĸ�ʽ:0x5A5A5A5A + ���� + �����ַ���
}__attribute__((aligned(4)))device_name_t;

//��¼m_weight_record������,ע��һ��Ҫ4�ֽڶ���
typedef struct
{
  uint8_t     data[DATA_FLASH_LEN_WORDS*4];    //�������ݴ洢�ĸ�ʽ:0x5B5B5B5B + ���� + �����ַ���
}__attribute__((aligned(4)))weight_data_t;


extern weight_data_info_t weight_data_info;

void ble_gatt_params_on_ble_evt(ble_evt_t const * p_ble_evt);
void advertising_start(void);
void advertising_init(void);
ret_code_t my_fds_init(void);
void device_name_set(void);
void weight_data_set(void);
void device_name_handle(void);
void weight_data_handle(void);
void ble_adv_start(void);
void ble_adv_stop(void);
void ble_adv_auto_start(void);
void ble_adv_auto_stop(void);
#endif

