/// @file      User/adc_task.cpp
/// @author    Hiroshi Mikuriya
/// @copyright Copyright© 2021 Hiroshi Mikuriya
///
/// DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.

#include "gpio.hpp"
#include "main.h"

namespace
{
/// DMAが書き込むバッファ
__I uint16_t s_dmaBuffer[COUNT_OF_MOTORS] = {0};
/// アナログ値
__IO uint16_t s_analog[COUNT_OF_MOTORS] = {0};
/// ADCペリフェラル
#define ADCx ADC2
/// DMAペリフェラル
#define DMAx DMA1
/// DMAチャネル
#define DMA_CHANNEL LL_DMA_CHANNEL_2
/// DMA割り込み番号
#define DMA_IRQn DMA1_Channel2_IRQn
} // namespace

extern "C"
{
  void adcTaskInit(void)
  {
    LL_DMA_ConfigAddresses(                                       //
        DMAx, DMA_CHANNEL,                                        //
        LL_ADC_DMA_GetRegAddr(ADCx, LL_ADC_DMA_REG_REGULAR_DATA), //
        reinterpret_cast<uint32_t>(s_dmaBuffer),                  //
        LL_DMA_DIRECTION_PERIPH_TO_MEMORY                         //
    );
    LL_DMA_SetDataLength(DMAx, DMA_CHANNEL, COUNT_OF_MOTORS);
    LL_DMA_EnableIT_TC(DMAx, DMA_CHANNEL);
    LL_DMA_EnableIT_TE(DMAx, DMA_CHANNEL);
    LL_DMA_EnableChannel(DMAx, DMA_CHANNEL);
    LL_ADC_Enable(ADCx);
    LL_ADC_REG_StartConversion(ADCx);
  }
  /// @brief ADCタスク<br>
  /// 可変抵抗の抵抗値を定期的に読み取りメッセージを送信する
  /// @param [in] res タスク共有リソース
  void adcTaskProc(TaskResource *res)
  {
    NVIC_DisableIRQ(DMA_IRQn);
    for (int i = 0; i < COUNT_OF_MOTORS; ++i)
    {
      res[i].analog = s_analog[i];
    }
    NVIC_EnableIRQ(DMA_IRQn);
  }
  /// @brief ADC変換完了DMA割り込み
  void adcCpltIRQ(void)
  {
    for (uint32_t i = 0; i < COUNT_OF_MOTORS; ++i)
    {
      s_analog[i] = s_dmaBuffer[i];
    }
  }
  /// @brief ADC変換エラーDMA割り込み
  void adcErrorIRQ(void)
  {
    // TODO
  }
}
