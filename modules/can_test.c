#include "main.h"
#include "can_test.h"


// 中科大 CAN 初始化程序
/**
 * @brief 初始化CAN总线
 * 
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
HAL_CAN_Start(hcan); //打开CAN
__HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //使能中断1
__HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING); //使能中断2
}


/*--------------------------------------------------------------------------------------------------*/
/**
 * @brief 发送数据帧
 * 
 * @param hcan   CAN编号
 * @param ID     ID
 * @param Data   被发送数据指针
 * @param Length 长度
 * @return uint_8 执行状态
 */

uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t used_mailbox;

  // 检测关键传参
  assert_param(hcan != NULL);

  tx_header.StdId = ID;
  tx_header.ExtId = 0;
  tx_header.IDE = 0;
  tx_header.RTR = 0;
  tx_header.DLC = Length; //数据长度

  return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}


/*--------------------------------------------------------------------------------------------------*/
/**
 * @brief 配置CAN过滤器
 * 
 * @param hcan         CAN编号 (hcan1, hcan2)
 * @param Object_Para  编号 | FIFOx | ID类型 | 帧类型  (这是使用方法，一坨或运算)
 * @param ID           接收 ID
 * @param Mask_ID      屏蔽 ID (0x3ff, 0x1fffffff)
 */

void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
  CAN_FilterTypeDef can_filter_init_structure;

  // 检测关键传参
  assert_param(hcan != NULL);

  if ((Object_Para & 0x02)) //判断是 标准帧还是扩展帧
  {
    // 标准帧
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ID << 3 >> 16;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
    // ID掩码值高16bit
    can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
    // ID掩码值低16bit
    can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
  }
  else
  {
    // 扩展帧
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ID << 5;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
    // ID掩码值高16bit
    can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
    // ID掩码值低16bit
    can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
  }

  // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
  can_filter_init_structure.FilterBank = Object_Para >> 3;
  // 滤波器绑定FIFOx, 只能绑定一个
  can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
  // 使能滤波器
  can_filter_init_structure.FilterActivation = ENABLE;
  // 滤波器模式, 设置ID掩码模式
  can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
  // 32位滤波
  can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
  can_filter_init_structure.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}



/*--------------------------------------------------------------------------------------------------*/
/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data;

  HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &header, &data);
  
  LED_Control(data);
}



/*--------------------------------------------------------------------------------------------------*/
void LED_Control(uint8_t data)
{
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,((data & 0b0001)==0b0001) ? GPIO_PIN_SET : GPIO_PIN_RESET); //三目运算符
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,((data & 0b0010)==0b0010) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,((data & 0b0100)==0b0100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
